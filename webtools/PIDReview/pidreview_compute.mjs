/**
 * Pi-side PIDReview compute (Node.js).
 *
 * Purpose:
 * - Parse an ArduPilot DataFlash .bin using the existing WebTools parser
 * - Compute time-domain series (Tar/Act/Out) and the step response estimate (Wiener filter)
 *   using the same math as the in-browser PIDReview tool.
 *
 * Output: JSON to stdout (for `fc_log_service.py` to embed into HTML).
 */
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";
import vm from "node:vm";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

function fatal(msg) {
  process.stderr.write(String(msg) + "\n");
  process.exit(2);
}

const logPath = process.argv[2];
if (!logPath) {
  fatal("Usage: node pidreview_compute.mjs <path-to-log.bin>");
}

function loadScriptToGlobal(jsFilePath) {
  const code = fs.readFileSync(jsFilePath, "utf8");
  vm.runInThisContext(code, { filename: jsFilePath });
}

// Load WebTools libs into the global scope (they define plain functions).
const webtoolsRoot = path.resolve(__dirname, "..");
loadScriptToGlobal(path.join(webtoolsRoot, "modules", "fft.js", "dist", "fft.js"));      // defines global FFTJS
loadScriptToGlobal(path.join(webtoolsRoot, "Libraries", "Array_Math.js"));
loadScriptToGlobal(path.join(webtoolsRoot, "Libraries", "fft.js"));

// Import DataflashParser (patched to be Node-safe).
const DataflashParserMod = await import(
  pathToFileURL(path.join(webtoolsRoot, "modules", "JsDataflashParser", "parser.js")).toString()
);
const DataflashParser = DataflashParserMod.default;

// -----------------------------
// Minimal helpers copied from PIDReview.js (kept identical where possible)
// -----------------------------
function get_PID_param_names(prefix) {
  return {
    P: prefix + "P",
    I: prefix + "I",
    D: prefix + "D",
    FF: prefix + "FF",
    D_FILT: prefix + "D_FILT",
    FLTT: prefix + "FLTT",
    FLTD: prefix + "FLTD",
  };
}

function split_into_batches(PID_log_messages, index, time) {
  const len = time.length;
  PID_log_messages[index].start_time = time[0];
  PID_log_messages[index].end_time = time[len - 1];
  if ((PID_log_messages.start_time == null) || (PID_log_messages[index].start_time < PID_log_messages.start_time)) {
    PID_log_messages.start_time = PID_log_messages[index].start_time;
  }
  if ((PID_log_messages.end_time == null) || (PID_log_messages[index].end_time > PID_log_messages.end_time)) {
    PID_log_messages.end_time = PID_log_messages[index].end_time;
  }

  let sample_rate_sum = 0;
  let sample_rate_count = 0;

  const ret = [];

  // Batch boundaries are based on parameter-set boundaries (derived from PARM)
  let param_set = 0;
  let set_start = PID_log_messages[index].params.sets[0].start_time;
  let set_end = PID_log_messages[index].params.sets[0].end_time;
  let batch_start = 0;

  for (let j = 1; j < len; j++) {
    const t = time[j];
    const is_set_boundary = (t < set_start) || (t > set_end);
    if (!is_set_boundary) {
      continue;
    }

    const count = (j - batch_start);
    if (count > 1) {
      const sample_rate = 1 / ((time[j - 1] - time[batch_start]) / count);
      sample_rate_sum += sample_rate;
      sample_rate_count++;
      ret.push({ param_set, sample_rate, batch_start, batch_end: j - 1 });
    }

    // Advance param set
    while (t > set_end && param_set + 1 < PID_log_messages[index].params.sets.length) {
      param_set++;
      set_start = PID_log_messages[index].params.sets[param_set].start_time;
      set_end = PID_log_messages[index].params.sets[param_set].end_time;
    }
    batch_start = j;
  }

  // Final batch
  const last_count = (len - batch_start);
  if (last_count > 1) {
    const sample_rate = 1 / ((time[len - 1] - time[batch_start]) / last_count);
    sample_rate_sum += sample_rate;
    sample_rate_count++;
    ret.push({ param_set, sample_rate, batch_start, batch_end: len - 1 });
  }

  // Store average sample rate on sets (same shape as browser tool expects)
  if (sample_rate_sum > 0 && sample_rate_count > 0) {
    PID_log_messages[index].average_sample_rate = sample_rate_sum / sample_rate_count;
  }
  return ret;
}

function find_start_index(time) {
  // Browser tool finds start of step windows. Keep conservative: first valid sample.
  for (let i = 0; i < time.length; i++) {
    if (Number.isFinite(time[i])) return i;
  }
  return 0;
}

function find_end_index(time) {
  // Browser tool uses window span; here: allow full range.
  for (let i = time.length - 1; i >= 0; i--) {
    if (Number.isFinite(time[i])) return i;
  }
  return time.length - 1;
}

function to_fft_format(out_interleaved, C) {
  // C is [real[], imag[]]
  const len = C[0].length;
  for (let i = 0; i < len; i++) {
    out_interleaved[i * 2] = C[0][i];
    out_interleaved[i * 2 + 1] = C[1][i];
  }
}

function compute_pid_sets(log) {
  // Mirrors PIDReview.js load() message selection.
  const PID_log_messages = [
    { id: ["PIDR"], prefixes: ["ATC_RAT_RLL_", "RLL_RATE_"] },
    { id: ["PIDP"], prefixes: ["ATC_RAT_PIT_", "PTCH_RATE_"] },
    { id: ["PIDY"], prefixes: ["ATC_RAT_YAW_", "YAW_RATE_"] },
    { id: ["PIQR"], prefixes: ["Q_A_RAT_RLL_"] },
    { id: ["PIQP"], prefixes: ["Q_A_RAT_PIT_"] },
    { id: ["PIQY"], prefixes: ["Q_A_RAT_YAW_"] },
    { id: ["RATE", "R"], prefixes: ["ATC_RAT_RLL_", "Q_A_RAT_RLL_"] },
    { id: ["RATE", "P"], prefixes: ["ATC_RAT_PIT_", "Q_A_RAT_PIT_"] },
    { id: ["RATE", "Y"], prefixes: ["ATC_RAT_YAW_", "Q_A_RAT_YAW_"] },
  ];

  PID_log_messages.have_data = false;
  for (const m of PID_log_messages) m.have_data = false;

  const US2S = 1 / 1000000;
  const TimeUS_to_seconds = (TimeUS) => array_scale(TimeUS, US2S);

  const PARM = log.get("PARM");
  for (let i = 0; i < PID_log_messages.length; i++) {
    PID_log_messages[i].params = { prefix: null, sets: [] };
    for (const prefix of PID_log_messages[i].prefixes) {
      const names = get_PID_param_names(prefix);
      let param_values = { start_time: 0 };
      for (const name in names) param_values[name] = null;

      let found_param = false;
      let last_set_end;
      for (let j = 0; j < PARM.Name.length; j++) {
        const param_name = PARM.Name[j];
        for (const [name, param_string] of Object.entries(names)) {
          if (param_name !== param_string) continue;
          const t = PARM.TimeUS[j] * US2S;
          const value = PARM.Value[j];
          found_param = true;
          if (param_values[name] != null && (param_values[name] != value)) {
            if ((last_set_end == null) || (t - last_set_end > 1.0)) {
              last_set_end = t;
              PID_log_messages[i].params.sets.push(Object.assign({}, param_values, { end_time: last_set_end }));
              param_values.start_time = t;
            } else {
              param_values[name] = value;
              param_values.start_time = t;
            }
          }
          param_values[name] = value;
          break;
        }
      }
      if (found_param) {
        PID_log_messages[i].params.sets.push(Object.assign({}, param_values, { end_time: Infinity }));
        PID_log_messages[i].params.prefix = prefix;
        break;
      }
    }
  }

  PID_log_messages.start_time = null;
  PID_log_messages.end_time = null;

  for (let i = 0; i < PID_log_messages.length; i++) {
    if (PID_log_messages[i].params.prefix == null) continue;
    const id = PID_log_messages[i].id[0];
    if (!(id in log.messageTypes)) continue;

    const log_msg = log.get(id);
    const is_RATE_msg = id === "RATE";
    const time = TimeUS_to_seconds(log_msg.TimeUS);
    const batches = split_into_batches(PID_log_messages, i, time);
    if (batches.length <= 0) continue;

    PID_log_messages[i].sets = [];
    for (const batch of batches) {
      if (PID_log_messages[i].sets[batch.param_set] == null) PID_log_messages[i].sets[batch.param_set] = [];
      if (is_RATE_msg) {
        const axis_prefix = PID_log_messages[i].id[1];
        PID_log_messages[i].sets[batch.param_set].push({
          time: time.slice(batch.batch_start, batch.batch_end),
          sample_rate: batch.sample_rate,
          Tar: Array.from(log_msg[axis_prefix + "Des"].slice(batch.batch_start, batch.batch_end)),
          Act: Array.from(log_msg[axis_prefix].slice(batch.batch_start, batch.batch_end)),
          Out: Array.from(log_msg[axis_prefix + "Out"].slice(batch.batch_start, batch.batch_end)),
        });
      } else {
        const rad2deg = 180.0 / Math.PI;
        PID_log_messages[i].sets[batch.param_set].push({
          time: time.slice(batch.batch_start, batch.batch_end),
          sample_rate: batch.sample_rate,
          Tar: array_scale(Array.from(log_msg.Tar.slice(batch.batch_start, batch.batch_end)), rad2deg),
          Act: array_scale(Array.from(log_msg.Act.slice(batch.batch_start, batch.batch_end)), rad2deg),
          Err: array_scale(Array.from(log_msg.Err.slice(batch.batch_start, batch.batch_end)), rad2deg),
          P: Array.from(log_msg.P.slice(batch.batch_start, batch.batch_end)),
          I: Array.from(log_msg.I.slice(batch.batch_start, batch.batch_end)),
          D: Array.from(log_msg.D.slice(batch.batch_start, batch.batch_end)),
          FF: Array.from(log_msg.FF.slice(batch.batch_start, batch.batch_end)),
        });
      }
    }

    // Compute Out for PID msgs (same as browser tool).
    for (const set of PID_log_messages[i].sets) {
      if (!set) continue;
      for (const batch of set) {
        if ("Out" in batch) continue;
        const len = batch.P.length;
        batch.Out = new Array(len);
        for (let k = 0; k < len; k++) {
          batch.Out[k] = batch.P[k] + batch.I[k] + batch.D[k] + batch.FF[k];
        }
      }
    }

    PID_log_messages[i].have_data = true;
    PID_log_messages.have_data = true;
  }

  return PID_log_messages;
}

function run_batch_fft_no_dom(data_set, window_size) {
  if (!Number.isInteger(Math.log2(window_size))) {
    throw new Error("Window size must be a power of two");
  }
  const num_sets = data_set.length;
  const window_overlap = 0.5;
  const window_spacing = Math.round(window_size * (1 - window_overlap));
  const windowing_function = hanning(window_size);
  const window_correction = window_correction_factors(windowing_function);
  const fft = new FFTJS(window_size);

  let sample_rate_sum = 0;
  let sample_rate_count = 0;
  for (let j = 0; j < num_sets; j++) {
    if (data_set[j] == null) continue;
    const num_batch = data_set[j].length;
    for (let i = 0; i < num_batch; i++) {
      if (data_set[j][i].Tar.length < window_size) continue;
      sample_rate_count++;
      sample_rate_sum += data_set[j][i].sample_rate;
    }
  }
  if (sample_rate_sum === 0) return null;
  const sample_time = sample_rate_count / sample_rate_sum;

  for (let j = 0; j < num_sets; j++) {
    if (data_set[j] == null) continue;
    let have_data = false;
    const num_batch = data_set[j].length;
    for (let i = 0; i < num_batch; i++) {
      if (data_set[j][i].Tar.length < window_size) continue;
      const ret = run_fft(data_set[j][i], ["Tar", "Act", "Out"], window_size, window_spacing, windowing_function, fft);
      if (!have_data) {
        have_data = true;
        data_set[j].FFT = { time: [], Tar: [], Act: [], Out: [] };
      }
      data_set[j].FFT.time.push(...array_offset(array_scale(ret.center, sample_time), data_set[j][i].time[0]));
      data_set[j].FFT.Tar.push(...ret.Tar);
      data_set[j].FFT.Act.push(...ret.Act);
      data_set[j].FFT.Out.push(...ret.Out);
    }
  }

  // Store shared FFT metadata on the parent `sets` array (matches browser shape).
  data_set.FFT = {
    average_sample_rate: 1 / sample_time,
    window_size,
    bins: rfft_freq(window_size, sample_time),
    window_correction,
  };
  return data_set.FFT;
}

function compute_step_response(PID, window_size_default = 512) {
  if (!PID || !PID.sets || PID.sets.length === 0) return null;

  // Ensure we have a baseline FFT metadata to supply sample rate / bins.
  if (!PID.sets.FFT) {
    run_batch_fft_no_dom(PID.sets, window_size_default);
  }
  if (!PID.sets.FFT) return null;

  const num_sets = PID.sets.length;
  const window_size = PID.sets.FFT.window_size;
  const real_len = real_length(window_size);
  const fft = new FFTJS(window_size);
  const transfer_function = fft.createComplexArray();
  const impulse_response = fft.createComplexArray();
  const windowing_function = hanning(window_size);
  const window_spacing = Math.round(window_size / 16);

  const sample_time = 1 / PID.sets.FFT.average_sample_rate;
  const step_end_index = Math.min(Math.ceil(0.5 / sample_time), window_size);
  const time = new Array(step_end_index);
  for (let j = 0; j < step_end_index; j++) time[j] = j * sample_time;

  // Noise estimate (same as browser tool).
  const cutfreq = 25;
  let len_lpf = PID.sets.FFT.bins.findIndex((x) => x > cutfreq);
  len_lpf += len_lpf - 2;
  const radius = Math.ceil(len_lpf * 0.5);
  const sigma = len_lpf / 6.0;
  let sn = new Array(real_len).fill(1.0);
  let last_sn = 0;
  for (let j = 0; j < len_lpf; j++) {
    sn[j] = last_sn + Math.exp((-0.5 / sigma ** 2) * (j - radius) ** 2);
    last_sn = sn[j];
  }
  for (let j = 0; j < len_lpf; j++) sn[j] /= last_sn;
  sn = [...sn, ...sn.slice(1, real_len - 1).reverse()];
  sn = array_scale(array_offset(array_scale(sn, -1.0), 1 + 1e-9), 10.0);
  sn = array_inverse(sn);

  const results = [];
  for (let j = 0; j < num_sets; j++) {
    if (PID.sets[j] == null) continue;
    const num_batch = PID.sets[j].length;
    let Step_mean = new Array(step_end_index).fill(0);
    let mean_count = 0;
    const individual = [];

    for (let i = 0; i < num_batch; i++) {
      if (PID.sets[j][i].Tar.length < window_size) continue;

      const FFT_res = run_fft(PID.sets[j][i], ["Tar", "Act"], window_size, window_spacing, windowing_function, fft, true);
      const fft_time = array_offset(array_scale(FFT_res.center, sample_time), PID.sets[j][i].time[0]);
      const start_index = find_start_index(fft_time);
      const end_index = find_end_index(fft_time) + 1;

      for (let k = start_index; k < end_index; k++) {
        if (FFT_res.TarMax[k] < 20.0) continue;

        const X = to_double_sided(FFT_res.Tar[k]);
        const Y = to_double_sided(FFT_res.Act[k]);
        const Xcon = complex_conj(X);
        const Pyx = complex_mul(Y, Xcon);
        let Pxx = complex_mul(X, Xcon);
        Pxx[0] = array_add(Pxx[0], sn);
        const H = complex_div(Pyx, Pxx);
        to_fft_format(transfer_function, H);
        fft.inverseTransform(impulse_response, transfer_function);

        const step = new Array(step_end_index);
        step[0] = impulse_response[0];
        for (let l = 1; l < step_end_index; l++) {
          step[l] = step[l - 1] + impulse_response[l * 2];
        }

        mean_count += 1;
        Step_mean = array_add(Step_mean, step);
        individual.push(step);
      }
    }

    if (mean_count <= 0) continue;
    results.push({
      set_index: j,
      time,
      mean: array_scale(Step_mean, 1 / mean_count),
      count: mean_count,
      // Keep individual steps but cap to reduce payload
      individual: individual.slice(0, 25),
    });
  }

  return results.length ? results : null;
}

// -----------------------------
// Main
// -----------------------------
const raw = fs.readFileSync(logPath);
const u8 = new Uint8Array(raw.buffer, raw.byteOffset, raw.byteLength);
const arrayBuffer = u8.buffer.slice(u8.byteOffset, u8.byteOffset + u8.byteLength);

const log = new DataflashParser();
log.processData(arrayBuffer, []);

const PID_log_messages = compute_pid_sets(log);
if (!PID_log_messages.have_data) {
  fatal("No PID or RATE log messages found in this log.");
}

const windowSize = 512;

// Build output per axis message entry.
const out = {
  log_path: logPath,
  window_size: windowSize,
  generated_at_utc: new Date().toISOString(),
  axes: [],
};

for (const pid of PID_log_messages) {
  if (!pid.have_data) continue;

  // Ensure FFT metadata exists for step response.
  run_batch_fft_no_dom(pid.sets, windowSize);
  const step = compute_step_response(pid, windowSize);

  // Provide a compact time-series sample for “time response analysis” (first set, first batch).
  let timeSeries = null;
  try {
    const firstSet = (pid.sets || []).find((s) => Array.isArray(s) && s.length);
    const batch = firstSet && firstSet[0];
    if (batch && Array.isArray(batch.time)) {
      const stride = Math.max(1, Math.floor(batch.time.length / 2500));
      const t = [];
      const tar = [];
      const act = [];
      const outv = [];
      for (let i = 0; i < batch.time.length; i += stride) {
        t.push(batch.time[i]);
        tar.push(batch.Tar[i]);
        act.push(batch.Act[i]);
        outv.push(batch.Out[i]);
      }
      timeSeries = { t, tar, act, out: outv, stride };
    }
  } catch (_e) {
    timeSeries = null;
  }

  out.axes.push({
    id: pid.id,
    prefix: pid.params && pid.params.prefix,
    sets: pid.params && pid.params.sets ? pid.params.sets.length : 0,
    step_response: step,
    time_series: timeSeries,
  });
}

process.stdout.write(JSON.stringify(out));

