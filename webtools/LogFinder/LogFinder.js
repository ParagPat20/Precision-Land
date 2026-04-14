
var DataflashParser
import('../modules/JsDataflashParser/parser.js').then((mod) => { DataflashParser = mod.default });

var dirHandle
let remoteSourceUrl = null
let remoteInventoryUrl = null
let mavftpPollTimer = null

function fcLogApiOrigin() {
    const s = document.getElementById("server_url").value.trim() || document.getElementById("inventory_url").value.trim()
    if (!s.length) {
        return null
    }
    return new URL(s, window.location.href).origin
}

async function fetchMavftpStatus() {
    const origin = fcLogApiOrigin()
    if (!origin) {
        return null
    }
    const url = new URL("/api/mavftp-status", origin)
    const response = await fetch(url.toString())
    if (!response.ok) {
        return null
    }
    return await response.json()
}

function renderMavftpBanner(status) {
    const el = document.getElementById("mavftp_status_banner")
    if (!el) {
        return
    }
    const block = status && (status.busy === true || status.mavftp_active === true)
    if (!block) {
        el.style.display = "none"
        el.textContent = ""
        document.querySelectorAll("[data-fc-download-btn]").forEach((b) => {
            b.disabled = false
            b.removeAttribute("title")
        })
        return
    }
    el.style.display = "block"
    const elapsed = status.elapsed_sec != null ? ` ~${status.elapsed_sec}s` : ""
    el.textContent = (status.detail || "FC log / MAVFTP transfer in progress") + elapsed
    document.querySelectorAll("[data-fc-download-btn]").forEach((b) => {
        b.disabled = true
        b.title = "Wait for the current transfer to finish (single FC link)."
    })
}

async function refreshMavftpBanner() {
    try {
        const st = await fetchMavftpStatus()
        renderMavftpBanner(st)
    } catch (_e) {
        /* ignore poll errors */
    }
}

function startMavftpStatusPolling() {
    if (mavftpPollTimer != null) {
        return
    }
    refreshMavftpBanner()
    mavftpPollTimer = window.setInterval(refreshMavftpBanner, 2000)
}

async function load_file_source(source) {
    if (source == null) {
        throw new Error("No file source provided")
    }

    if (source.url != null) {
        const response = await fetch(source.url)
        if (!response.ok) {
            throw new Error(`Failed to fetch ${source.url}: HTTP ${response.status}`)
        }
        return await response.arrayBuffer()
    }

    return await source.arrayBuffer()
}

async function get_dir_then_load() {

    if (typeof(window.showDirectoryPicker) != "function") {
        alert("This browser does not support directory opening.")
        return
    }

    dirHandle = await window.showDirectoryPicker().catch(() => { })

    await load_from_dir()

    // Enable reload if valid dir
    document.getElementById("reload").disabled = dirHandle == null
}

function format_size_bytes(size) {
    const unit_array = ['B', 'kB', 'MB', 'GB', 'TB']
    const unit_index = (size == 0) ? 0 : Math.floor(Math.log(size) / Math.log(1024))
    const scaled_size = size / Math.pow(1024, unit_index)
    return scaled_size.toFixed(2) + " " + unit_array[unit_index]
}

function format_utc_time(time_utc) {
    if (!time_utc || time_utc <= 0) {
        return "Unknown time"
    }
    return new Date(time_utc * 1000).toLocaleString()
}

function render_inventory(logs, source, transferStatus) {
    const inventoryDiv = document.getElementById("inventory")
    inventoryDiv.replaceChildren()

    if (!Array.isArray(logs) || logs.length === 0) {
        const empty = document.createElement("p")
        empty.textContent = "No flight-controller logs were reported."
        inventoryDiv.appendChild(empty)
        return
    }

    const title = document.createElement("h3")
    title.textContent = "Flight Controller Logs"
    inventoryDiv.appendChild(title)

    const table = document.createElement("table")
    table.style.width = "1200px"
    table.style.borderCollapse = "collapse"

    const headerRow = document.createElement("tr")
    ;["Log ID", "Time", "Size", "State", "Action"].forEach((text) => {
        const th = document.createElement("th")
        th.textContent = text
        th.style.textAlign = "left"
        th.style.padding = "8px"
        th.style.borderBottom = "1px solid #ccc"
        headerRow.appendChild(th)
    })
    table.appendChild(headerRow)

    for (const log of logs) {
        const row = document.createElement("tr")
        if (log.is_latest) {
            row.style.backgroundColor = "#eef6ff"
        }

        const idCell = document.createElement("td")
        idCell.style.padding = "8px"
        idCell.textContent = log.is_latest ? `${log.id} (latest)` : `${log.id}`
        row.appendChild(idCell)

        const timeCell = document.createElement("td")
        timeCell.style.padding = "8px"
        timeCell.textContent = format_utc_time(log.time_utc)
        row.appendChild(timeCell)

        const sizeCell = document.createElement("td")
        sizeCell.style.padding = "8px"
        sizeCell.textContent = format_size_bytes(log.size)
        row.appendChild(sizeCell)

        const stateCell = document.createElement("td")
        stateCell.style.padding = "8px"
        stateCell.textContent = log.cached ? "Downloaded on RPi" : "Available on FC"
        row.appendChild(stateCell)

        const actionCell = document.createElement("td")
        actionCell.style.padding = "8px"
        const button = document.createElement("button")
        button.textContent = log.cached ? "Load downloaded log" : "Download to RPi"
        if (!log.cached) {
            button.setAttribute("data-fc-download-btn", "1")
            const block = transferStatus && (transferStatus.busy === true || transferStatus.mavftp_active === true)
            if (block) {
                button.disabled = true
                button.title = "MAVFTP / FC log transfer already running — wait for it to finish."
            }
        }
        button.addEventListener("click", () => {
            loading_call(async () => {
                const apiBase = new URL(source, window.location.href).toString()
                if (!log.cached) {
                    const st = await fetchMavftpStatus()
                    if (st && (st.busy === true || st.mavftp_active === true)) {
                        throw new Error("FC link busy: " + (st.detail || "download in progress"))
                    }
                    const response = await fetch(new URL(`/api/fc-logs/download/${log.id}`, apiBase))
                    if (!response.ok) {
                        throw new Error(`Failed to download log ${log.id}: HTTP ${response.status}`)
                    }
                    await response.json()
                }

                await load_from_server(document.getElementById("server_url").value.trim())
                await load_inventory_from_server(apiBase)
            })
        })
        actionCell.appendChild(button)
        row.appendChild(actionCell)

        table.appendChild(row)
    }

    inventoryDiv.appendChild(table)
}

async function load_inventory_from_server(url = null) {
    const input = document.getElementById("inventory_url")
    const source = url || input.value.trim()
    if (source.length == 0) {
        return
    }

    const resolvedSource = new URL(source, window.location.href).toString()
    remoteInventoryUrl = resolvedSource
    const response = await fetch(resolvedSource)
    if (!response.ok) {
        throw new Error(`Failed to fetch FC log inventory: HTTP ${response.status}`)
    }

    const manifest = await response.json()
    const transferStatus = await fetchMavftpStatus()
    render_inventory(manifest.logs || [], resolvedSource, transferStatus)
    renderMavftpBanner(transferStatus)
    startMavftpStatusPolling()
}

async function load_from_server(url = null) {
    const input = document.getElementById("server_url")
    const source = url || input.value.trim()
    if (source.length == 0) {
        return
    }

    remoteSourceUrl = source
    dirHandle = null
    document.getElementById("reload").disabled = false

    const start = performance.now()
    reset()
    document.title = "Remote logs: " + source

    let progress = document.getElementById("load")
    progress.parentElement.hidden = false

    const response = await fetch(source)
    if (!response.ok) {
        throw new Error(`Failed to fetch server log list: HTTP ${response.status}`)
    }

    const manifest = await response.json()
    const remoteLogs = manifest.logs || []
    progress.setAttribute("max", remoteLogs.length)

    let logs = {}
    for (let i = 0; i < remoteLogs.length; i++) {
        const remoteLog = remoteLogs[i]
        progress.setAttribute("value", i + 1)

        try {
            const remoteUrl = new URL(remoteLog.url, source).toString()
            const arrayBuffer = await load_file_source({ url: remoteUrl })
            let info = load_log(arrayBuffer)
            if (info == null) {
                console.log("Load failed: " + remoteLog.rel_path)
                continue
            }

            const remoteFile = {
                name: remoteLog.name,
                relativePath: remoteLog.rel_path || remoteLog.name,
                url: remoteUrl
            }
            info.name = remoteLog.name
            info.rel_path = remoteFile.relativePath
            logs[remoteFile.relativePath] = { info, fileHandle: remoteFile }
        } catch (error) {
            console.log("Remote load failed: " + remoteLog.name + " " + error)
        }
    }

    setup_table(logs)

    const end = performance.now()
    console.log(`Loaded ${Object.values(logs).length} remote logs in: ${end - start} ms`)
    progress.parentElement.hidden = true
}

async function load_from_dir() {
    remoteSourceUrl = null

    if (dirHandle == null) {
        return
    }

    const start = performance.now()

    reset()

    // Set page title to folder name
    document.title = "Logs in: " + dirHandle.name + "/"

    let progress = document.getElementById("load")
    progress.parentElement.hidden = false

    async function get_logs() {
        async function* getFilesRecursively(entry, path) {
            let relativePath
            if (path == null) {
                relativePath = entry.name
            } else {
                relativePath = path + "/" + entry.name
            }
            if (entry.kind === "file") {
                const file = await entry.getFile()
                if ((file !== null) && (file.name.toLowerCase().endsWith(".bin"))) {
                    file.relativePath = relativePath
                    yield file
                }
            } else if (entry.kind === "directory") {
                for await (const handle of entry.values()) {
                    try {
                        yield* getFilesRecursively(handle, relativePath)
                    } catch {
                        if (handle.kind === "directory") {
                            console.log("Opening " + handle.name + " in " + relativePath + " failed")
                        }
                    }
                }
            }
        }

        // Have handle, get logs

        // Do pre-count for progresses bar
        const count_start = performance.now()
        let count = 0
        for await (const fileHandle of getFilesRecursively(dirHandle)) {
            count++
        }
        console.log(`Counted ${count} logs in: ${performance.now() - count_start} ms`);
        progress.setAttribute("max", count)

        let logs = {}
        count = 0
        for await (const fileHandle of getFilesRecursively(dirHandle)) {
            count++
            progress.setAttribute("value", count)

            // Helper to allow waiting for file to load
            const wait_for_load = () => {
                const reader = new FileReader()
                return new Promise((resolve, reject) => {
                    reader.onerror = () => {
                        reader.abort()
                        reject(new DOMException("Problem parsing input file."))
                    }

                    reader.onload = () => {
                        let info = load_log(reader.result)
                        if (info == null) {
                            console.log("Load failed: " + fileHandle.relativePath)
                        } else {
                            info.name = fileHandle.name
                            info.rel_path = fileHandle.relativePath
                            logs[fileHandle.relativePath] = { info, fileHandle}
                        }
                        resolve()
                    }
                    reader.readAsArrayBuffer(fileHandle)
                })
            }

            await wait_for_load()
        }
        return logs
    }

    let logs = await get_logs()

    setup_table(logs)

    const end = performance.now();
    console.log(`Loaded ${Object.values(logs).length} logs in: ${end - start} ms`);
    progress.parentElement.hidden = true

}

let tables = []
function setup_table(logs) {

    // Sort in to sections based on hardware ID
    let boards = {}
    for (const log of Object.values(logs)) {
        let key = "Unknown"
        if (log.info.fc_string != null) {
            key = log.info.fc_string
        }
        if (!(key in boards)) {
            boards[key] = []
        }
        boards[key].push(log)
    }

    const tables_div = document.getElementById("tables")

    for (const [board_id, board_logs] of Object.entries(boards)) {

        function get_common_path(board_logs) {
            const first_path = board_logs[0].fileHandle.relativePath
            // Loop through the letters of the first string
            for (let i = 0; i <= first_path.length; i++) {
                // Loop through the other strings
                for (let j = 1; j < board_logs.length; j++) {
                    // Check if this character is also present in the same position of each string
                    if (first_path[i] !== board_logs[j].fileHandle.relativePath[i]) {
                        // If not, return the string up to and including the previous character
                        return first_path.slice(0, i);
                    }
                }
            }
            return first_path
        }
        const common_path = get_common_path(board_logs)

        const board_details = document.createElement("details")
        board_details.setAttribute("open", true);
        tables_div.appendChild(board_details)

        const board_summary = document.createElement("summary")
        board_summary.appendChild(document.createTextNode(board_id))
        if (common_path.length > 0) {
            board_summary.appendChild(document.createTextNode(": " + common_path))
        }
        board_summary.style.fontSize = " 1.17em"
        board_summary.style.fontWeight = "bold"
        board_details.style.marginBottom = "15px"
        board_details.appendChild(board_summary)

        board_details.appendChild(document.createElement("br"))

        const table_div = document.createElement("div")
        table_div.style.width = "1200px"
        board_details.appendChild(table_div)

        // custom formatter to add a param download button
        function param_download_button(cell, formatterParams, onRendered) {

            function save_parameters() {
                const log = cell.getRow().getData()
                const params = log.info.params

                if (Object.keys(params).length == 0) {
                    return
                }

                // Get contents of file to download
                const text = get_param_download_text(params)

                // make sure there are no slashes in file name
                let log_file_name = log.info.name.replace(/.*[\/\\]/, '')

                // Replace the file extension
                const file_name = (log_file_name.substr(0, log_file_name.lastIndexOf('.')) || log_file_name) + ".param"

                // Save
                var blob = new Blob([text], { type: "text/plain;charset=utf-8" })
                saveAs(blob, file_name)
            }

            let button = document.createElement("input")
            button.setAttribute('value', 'Parameters')
            button.setAttribute('type', 'button')
            button.addEventListener("click", save_parameters)
            button.disabled = Object.keys(cell.getRow().getData().info.params).length == 0

            return button
        }

        // custom formatter to add a open in button
        function open_in_button(cell, formatterParams, onRendered) {

            // Button to hold tool tip
            let button = document.createElement("input")
            button.setAttribute('value', 'Open In')
            button.setAttribute('type', 'button')

            function get_file_fun() {
                return cell.getRow().getData().fileHandle
            }

            const open_in = get_open_in(get_file_fun)
            open_in.update_enable(cell.getRow().getData().info.available_log_messages)


            tippy(button, {
                content: open_in.tippy_div,
                placement: 'left',
                interactive: true,
                appendTo: () => document.body,
            })
            return button
        }

        // Formatter to add custom buttons
        function check_warnings(cell, formatterParams, onRendered) {
            const log = cell.getRow().getData()
            const arming_checks_disabled = (("ARMING_SKIPCHK" in log.info.params) && log.info.params.ARMING_SKIPCHK > 0) ||(("ARMING_CHECK" in log.info.params) && log.info.params.ARMING_CHECK === 0)
            if (!arming_checks_disabled && !log.info.watchdog && !log.info.crash_dump) {
                // Nothing to warn about
                return
            }

            let img = document.createElement("img")
            img.style.width = "20px"
            img.style.verticalAlign = "bottom"

            if (log.info.crash_dump || log.info.watchdog) {
                img.src = "../images/exclamation-triangle-red.svg"

            } else {
                // Arming checks 0
                img.src = "../images/exclamation-triangle-orange.svg"

            }

            let tippy_div = document.createElement("div")

            if (log.info.crash_dump) {
                const para = document.createElement("p")
                tippy_div.appendChild(para)
                para.appendChild(document.createTextNode("Crash Dump file detected."))
                para.appendChild(document.createElement("br"))
                para.appendChild(document.createTextNode("For more information see ArduPilot "))

                const link = document.createElement("a")
                link.href = "https://ardupilot.org/copter/docs/common-watchdog.html#crash-dump"
                link.appendChild(document.createTextNode("documentation"))
                link.style="color:red;"

                para.appendChild(link)
                para.appendChild(document.createTextNode("."))
            }

            if (log.info.watchdog) {
                const para = document.createElement("p")
                tippy_div.appendChild(para)
                para.appendChild(document.createTextNode("Watchdog reboot detected."))
                para.appendChild(document.createElement("br"))
                para.appendChild(document.createTextNode("For more information see ArduPilot "))
            
                const link = document.createElement("a")
                link.href = "https://ardupilot.org/copter/docs/common-watchdog.html#independent-watchdog-and-crash-dump"
                link.appendChild(document.createTextNode("documentation"))
                link.style="color:red;"

                para.appendChild(link)
                para.appendChild(document.createTextNode("."))
            }

            if (arming_checks_disabled) {
                const para = document.createElement("p")
                tippy_div.appendChild(para)
                para.appendChild(document.createTextNode("Arming checks disabled."))
            }

            tippy(img, {
                content: tippy_div,
                placement: 'left',
                interactive: true,
                appendTo: () => document.body,
            })

            return img
        }

        // Formatter to add custom buttons
        function buttons(cell, formatterParams, onRendered) {
            let div = document.createElement("div")
            div.appendChild(param_download_button(cell, formatterParams, onRendered))
            div.appendChild(document.createTextNode(" "))
            div.appendChild(open_in_button(cell, formatterParams, onRendered))

            const warning = check_warnings(cell, formatterParams, onRendered)
            if (warning != null) {
                div.appendChild(document.createTextNode(" "))
                div.appendChild(warning)
            }
            return div
        }

        // Name formatter to add path on tooltip
        function name_format(cell, formatterParams, onRendered) {
            let div = document.createElement("div")
            const file = cell.getRow().getData().fileHandle
            if (file == null) {
                return
            }
            div.appendChild(document.createTextNode(file.name))

            tippy(div, {
                content: file.relativePath,
                interactive: true,
                appendTo: () => document.body,
            })

            return div
        }

        // Make file size a nice string with units
        function size_format(cell, formatterParams, onRendered) {
            const size = cell.getRow().getData().info.size
            const unit_array = ['B', 'kB', 'MB', 'GB', 'TB']
            const unit_index = (size == 0) ? 0 : Math.floor(Math.log(size) / Math.log(1024))
            const scaled_size = size / Math.pow(1024, unit_index)
            return scaled_size.toFixed(2) + " " + unit_array[unit_index]
        }

        // Make flight time a nice string with units
        function flight_time_format(cell, formatterParams, onRendered) {
            const flight_time = cell.getRow().getData().info.flight_time
            if (flight_time == null) {
                return "Unknown"
            }

            // Try human readable
            if (flight_time == 0) {
                return "-"
            }
            const dur = luxon.Duration.fromMillis(flight_time * 1000)
            return dur.rescale().toHuman({listStyle: 'narrow', unitDisplay: 'short'})

            // Might like this better, not sure
            //return dur.toFormat("hh:mm:ss")
        }

        // Format a param diff count and popup with details
        function param_diff_format(cell, formatterParams, onRendered) {
            const diff = cell.getRow().getData().param_diff
            if (diff == null) {
                return "-"
            }

            const count = Object.keys(diff.added).length + Object.keys(diff.missing).length + Object.keys(diff.changed).length
            const count_div = document.createElement("div")
            count_div.appendChild(document.createTextNode(count))

            let tippy_div = document.createElement("div")
            if (count == 0) {
                tippy_div.appendChild(document.createTextNode("No change"))

            } else {
                tippy_div.style.width = '325px'
                tippy_div.style.maxHeight = '90vh'
                tippy_div.style.overflow = 'auto'

                // Sort alphabetically, localeCompare does underscores differently to built in sort
                function param_sort(a, b) {
                    return a.localeCompare(b, undefined, {numeric: true})
                }

                if (Object.keys(diff.added).length > 0) {


                    const details = document.createElement("details")
                    details.setAttribute("open", true);
                    details.style.marginBottom = "5px"
                    tippy_div.appendChild(details)

                    const summary = document.createElement("summary")
                    summary.appendChild(document.createTextNode("New:"))
                    details.appendChild(summary)

                    for (const name of Object.keys(diff.added).sort(param_sort)) {
                        const text = name + ": " + param_to_string(diff.added[name])
                        details.appendChild(document.createTextNode(text))
                        details.appendChild(document.createElement("br"))
                    }
                }

                if (Object.keys(diff.missing).length > 0) {
                    const details = document.createElement("details")
                    details.setAttribute("open", true);
                    details.style.marginBottom = "5px"
                    tippy_div.appendChild(details)

                    const summary = document.createElement("summary")
                    summary.appendChild(document.createTextNode("Missing:"))
                    details.appendChild(summary)

                    for (const name of Object.keys(diff.missing).sort(param_sort)) {
                        const text = name + ": " + param_to_string(diff.missing[name])
                        details.appendChild(document.createTextNode(text))
                        details.appendChild(document.createElement("br"))
                    }
                }

                if (Object.keys(diff.changed).length > 0) {
                    const details = document.createElement("details")
                    details.setAttribute("open", true);
                    details.style.marginBottom = "5px"
                    tippy_div.appendChild(details)

                    const summary = document.createElement("summary")
                    summary.appendChild(document.createTextNode("Changed:"))
                    details.appendChild(summary)

                    for (const name of Object.keys(diff.changed).sort(param_sort)) {
                        const text = name + ": " + param_to_string(diff.changed[name].from) + " => " + param_to_string(diff.changed[name].to)
                        details.appendChild(document.createTextNode(text))
                        details.appendChild(document.createElement("br"))
                    }
                }
            }

            tippy(count_div, {
                content: tippy_div,
                maxWidth: '350px',
                placement: 'left',
                interactive: true,
                appendTo: () => document.body,
            })

            return count_div
        }

        // Find the param diff between the first and last row
        function total_param_diff_calc(values, data, calcParams) {
            const len = data.length
            if (len <= 1) {
                return null
            }
            return get_param_diff(data[len-1].info.params, data[0].info.params)
        }

        function get_dist_string(cell) {
            const dist_m = cell.getRow().getData().info.distance_traveled
            if (dist_m == null) {
                return "-"
            }
            if (dist_m < 2000) {
                return dist_m.toFixed(2) + " m"
            }
            const dist_km = dist_m / 1000.0
            return dist_km.toFixed(2) + " km"
        }

        function distance_format(cell, formatterParams, onRendered) {

            if (cell.getRow().getData().info.distance_traveled == null) {
                return "-"
            }

            const div = document.createElement("div")
            div.appendChild(document.createTextNode(get_dist_string(cell)))

            // Add map tool tip
            function tippy_show(instance) {

                if (instance.props.content !== "") {
                    // Content already loaded
                    return
                }

                let tippy_div = document.createElement("div")
                instance.setContent(tippy_div)

                tippy_div.style.width = '500px';
                tippy_div.style.height = '500px';

                let map = L.map(tippy_div)

                L.tileLayer('http://{s}.tile.osm.org/{z}/{x}/{y}.png', {
                    attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors'
                }).addTo(map)

                async function process_data(data) {
                    let log = new DataflashParser()
                    log.processData(data, [])

                    if (!('POS' in log.messageTypes)) {
                        return
                    }

                    const lat = log.get('POS', 'Lat')
                    const lng = log.get('POS', 'Lng')

                    const len = lat.length
                    const latlngs = new Array(len)
                    for (let i = 0; i<len; i++) {
                        latlngs[i] = [lat[i] * 1e-7, lng[i] * 1e-7]
                    }

                    var polyline = L.polyline(latlngs).addTo(map)

                    map.fitBounds(polyline.getBounds());
                }

                const file = cell.getRow().getData().fileHandle
                if (file.url != null) {
                    fetch(file.url)
                        .then((response) => response.arrayBuffer())
                        .then((data) => process_data(data))
                } else {
                    const reader = new FileReader()
                    reader.onload = function (e) {
                        process_data(reader.result)
                    }
                    reader.readAsArrayBuffer(file)
                }
            }

            if (typeof L !== 'undefined') {
                // Add map tool tip if leaflet is available
                tippy(div, {
                    maxWidth: '750px',
                    placement: 'left',
                    delay: [500, 0],
                    interactive: true,
                    appendTo: () => document.body,
                    onShow: tippy_show
                })
            }

            return div
        }

        // Tabulator will still show the bottom row if there is only one data row
        // manually disable in this case
        const multiple_rows = board_logs.length > 1
        const size_bottom_calc = multiple_rows ? "sum" : false
        const flight_time_bottom_calc = multiple_rows ? "sum" : false
        const flight_distance_bottom_calc = multiple_rows ? "sum" : false
        const param_diff_bottom_calc = multiple_rows ? total_param_diff_calc : false

        const table = new Tabulator(table_div, {
            height: "fit-content",
            data: board_logs,
            index: "info.rel_path",
            layout: "fitColumns",
            columns: [
                {
                    title: "Date",
                    field: "info.time_stamp",
                    width: 160,
                    formatter:"datetime",
                    formatterParams: {
                        outputFormat: "dd/MM/yyyy hh:mm:ss a",
                        invalidPlaceholder: "No GPS",
                    },
                    sorter:"datetime",
                },
                { title: "Name", field: "info.name", formatter:name_format },
                { title: "Size", field: "info.size", formatter:size_format, bottomCalc:size_bottom_calc, bottomCalcFormatter:size_format, width: 90 },
                { title: "Firmware Version", field:"info.fw_string" },
                { title: "Flight Time", field:"info.flight_time", formatter:flight_time_format, bottomCalc:flight_time_bottom_calc, bottomCalcFormatter:flight_time_format, width: 105 },
                { title: "Flight distance", field:"info.distance_traveled", formatter:distance_format, bottomCalc:flight_distance_bottom_calc, bottomCalcFormatter:get_dist_string, width: 125 },
                { title: "Param Changes", field:"param_diff", formatter:param_diff_format, headerSort:false, width: 110, bottomCalc:param_diff_bottom_calc, bottomCalcFormatter:param_diff_format },
                { title: "" , headerSort:false, formatter:buttons, width: 185 },
            ],
            initialSort: [
                { column:"info.time_stamp", dir:"asc"},
            ]
        })

        table.on("dataSorted", (sorters, rows) => { update_param_diff(table, rows) })

        tables.push(table)
    }

}

// Calculate the diff between two param sets
function get_param_diff(params, prev_params) {
    // Superset of param names from both files
    const names = new Set([...Object.keys(prev_params), ...Object.keys(params)])

    // Do param diff
    let added = {}
    let missing = {}
    let changed = {}
    for (const name of names) {
        const have_old = name in prev_params
        const have_new = name in params
        if (have_new && !have_old) {
            // Only in new
            added[name] = params[name]

        } else if (!have_new && have_old) {
            // Only in old
            missing[name] = prev_params[name]

        } else if (prev_params[name] != params[name]) {
            // In both with different value

            // Check if this change should be ignored
            let show_change = true
            for (const ignore of param_diff_ignore) {
                if (ignore.check.checked && ignore.fun(name)) {
                    show_change = false
                    break
                }
            }

            if (show_change) {
                changed[name] = { from: prev_params[name], to: params[name]}
            }
        }
    }

    return { added, missing, changed }
}

function update_param_diff(table, rows) {
    // Never any change in first row
    rows[0].getData().param_diff = null

    // Calculate diff for each row
    for (let i = 1; i<rows.length; i++) {
        const param_diff = get_param_diff(rows[i].getData().info.params, rows[i-1].getData().info.params)
        rows[i].getData().param_diff = param_diff
    }

    // Trigger re-format
    table.redraw(true)
}

function load_log(log_file) {

    let log = new DataflashParser()
    try {
        log.processData(log_file, [])
    } catch {
        return
    }

    // Probably not a ArduPilot log
    if (Object.keys(log.messageTypes).length == 0) {
        return
    }

    const version = get_version_and_board(log)

    // Populate the board name from boards lookup
    let board_name
    if ((version.board_id != null) && (version.board_id in board_types)) {
        board_name = board_types[version.board_id]
    }

    // Get params, extract flight time
    let params = {}
    let start_flight_time
    let end_flight_time
    if ("PARM" in log.messageTypes) {
        const PARM = log.get("PARM")
        for (let i = 0; i < PARM.Name.length; i++) {
            const name = PARM.Name[i]
            const value = PARM.Value[i]
            params[name] = value

            // Check for cumulative flight time, get first and last value
            if (name == "STAT_FLTTIME") {
                if (start_flight_time == null) {
                    start_flight_time = value
                }
                end_flight_time = value
            }
        }
    }

    let flight_time
    if (start_flight_time != null) {
        flight_time = end_flight_time - start_flight_time
    }

    // Get start time, convert to luxon format to work with table
    const time_stamp = luxon.DateTime.fromJSDate(log.extractStartTime())

    // Check for bad things we should warn about
    const watchdog = 'WDOG' in log.messageTypes

    let crash_dump
    if ('FILE' in log.messageTypes) {
        crash_dump = false
        const names = log.get('FILE', 'FileName')
        const len = names.length
        for (let i = 0; i<len; i++) {
            if (names[i].endsWith("crash_dump.bin")) {
                crash_dump = true
                break
            }
        }
    }

    // Calculate distance traveled
    let distance_traveled = null
    if ('POS' in log.messageTypes) {
        distance_traveled = 0

        const lat = log.get('POS', 'Lat')
        const lng = log.get('POS', 'Lng')
        const alt = log.get('POS', 'Alt')

        function diff_longitude(lon1, lon2) {
            let dlon = lon1-lon2
            if (dlon > 1800000000) {
                dlon -= 3600000000
            } else if (dlon < -1800000000) {
                dlon += 3600000000
            }
            return dlon
        }

        function longitude_scale(lat) {
            const scale = Math.cos(lat * (1.0e-7 * (Math.PI / 180.0)))
            return Math.max(scale, 0.01)
        }

        const LATLON_TO_M = 0.011131884502145034

        const len = lat.length
        for (let i = 1; i<len; i++) {
            const x = (lat[i - 1] - lat[i]) * LATLON_TO_M
            const y = diff_longitude(lng[i - 1], lng[i]) * LATLON_TO_M * longitude_scale((lat[i - 1]+lat[i])/2)
            const z = alt[i - 1] - alt[i]

            distance_traveled += Math.sqrt(x**2 + y**2 + z**2)
        }
    }

    return {
        size: log_file.byteLength,
        fw_string: version.fw_string,
        git_hash: version.fw_hash,
        board_id: version.board_id,
        fc_string: version.flight_controller,
        os_string: version.os_string,
        board_name,
        build_type: version.build_type,
        params,
        time_stamp,
        flight_time,
        watchdog,
        crash_dump,
        distance_traveled,
        available_log_messages: get_base_log_message_types(log),
    }
}

function reset() {

    // Reset title
    document.title = "ArduPilot Log Finder"

    // Remove all tables
    document.getElementById("tables").replaceChildren()
    tables = []

    // Reset progress
    let progress = document.getElementById("load")
    progress.setAttribute("value", 0)
    progress.setAttribute("max", 0)
    progress.parentElement.hidden = true
}

async function reload_current_source() {
    if (remoteInventoryUrl != null) {
        await load_inventory_from_server(remoteInventoryUrl)
    }
    if (remoteSourceUrl != null) {
        await load_from_server(remoteSourceUrl)
        return
    }
    await load_from_dir()
}

let param_diff_ignore = [
    { name: "Statistics (STAT_)", fun: (name) => { return name.startsWith("STAT_") || (name == "SYS_NUM_RESETS") } },
    { name: "Gyro offsets", fun: (name) => { return /(:?(INS)[45]?_(GYR)[23]?(OFFS_)[XYZ])/gm.test(name)} },
    { name: "Gyro cal temperature", fun: (name) => { return /(:?(INS)[45]?(_GYR)[123]?(_CALTEMP))/gm.test(name)} },
    { name: "Baro ground pressure", fun: (name) => { return /(:?(BARO)[123]?(_GND_PRESS))/gm.test(name)} },
    { name: "Compass declination", fun: (name) => { return name == "COMPASS_DEC"} },
    { name: "Airspeed offset", fun: (name) => { return /(:?(ARSPD)[123]?(_OFFSET))/gm.test(name)} },
    { name: "Stream rates", fun: (name) => {
        return /(:?(SR)[0123456]_(RAW_SENS))/gm.test(name) ||
            /(:?(SR)[0123456]_(EXT_STAT))/gm.test(name) ||
            /(:?(SR)[0123456]_(RC_CHAN))/gm.test(name) ||
            /(:?(SR)[0123456]_(RAW_CTRL))/gm.test(name) ||
            /(:?(SR)[0123456]_(POSITION))/gm.test(name) ||
            /(:?(SR)[0123456]_(EXTRA1))/gm.test(name) ||
            /(:?(SR)[0123456]_(EXTRA2))/gm.test(name) ||
            /(:?(SR)[0123456]_(EXTRA3))/gm.test(name) ||
            /(:?(SR)[0123456]_(PARAMS))/gm.test(name) ||
            /(:?(SR)[0123456]_(ADSB))/gm.test(name)
        }
    },
]

let board_types = {}
async function initial_load() {

    document.getElementById("reload").disabled = true

    const fieldset = document.getElementById("param_diff_ignore")
    for (let i=0; i<param_diff_ignore.length; i++) {
        if (i > 0) {
            fieldset.appendChild(document.createTextNode(", "))
        }

        const ignore = param_diff_ignore[i]
        const id = "param_diff_ignore" + i

        ignore.check = document.createElement("input")
        ignore.check.setAttribute('type', 'checkbox')
        ignore.check.setAttribute('id', id)
        ignore.check.checked = true

        function redraw_tables() {
            for (const table of tables) {
                update_param_diff(table, table.getRows())
            }
        }
        ignore.check.addEventListener('change', redraw_tables)

        let label = document.createElement("label")
        label.setAttribute('for', id)
        label.appendChild(document.createTextNode(ignore.name))

        fieldset.appendChild(ignore.check)
        fieldset.appendChild(label)
    }

    function load_board_types(text) {
        const lines = text.match(/[^\r\n]+/g)
        for (const line of lines) {
            // This could be combined with the line split if I was better at regex's
            const match = line.match(/(^[-\w]+)\s+(\d+)/)
            if (match) {
                const board_id = match[2]
                let board_name = match[1]

                // Shorten name for readability
                board_name = board_name.replace(/^TARGET_HW_/, "")
                board_name = board_name.replace(/^EXT_HW_/, "")
                board_name = board_name.replace(/^AP_HW_/, "")

                board_types[board_id] = board_name
            }
        }
    }


    fetch("board_types.txt")
        .then((res) => {
        return res.text();
    }).then((data) => load_board_types(data));

    if (typeof(window.showDirectoryPicker) != "function") {
        const localButton = document.getElementById("get_dir")
        if (localButton != null) {
            localButton.disabled = true
            localButton.title = "This browser does not support local directory access. Use the server log URL instead."
        }
    }

    const params = new URLSearchParams(window.location.search)
    const source = params.get("source")
    const inventory = params.get("inventory")
    if (source != null) {
        document.getElementById("server_url").value = source
        loading_call(() => load_from_server(source))
    }
    if (inventory != null) {
        document.getElementById("inventory_url").value = inventory
        loading_call(() => load_inventory_from_server(inventory))
    }
    if (source != null || inventory != null) {
        startMavftpStatusPolling()
    }

}
