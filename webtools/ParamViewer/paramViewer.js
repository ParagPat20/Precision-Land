let table = null
let currentParams = {}
let editedValues = new Map()
let pendingWriteChanges = []

function apiUrl(path) {
    return new URL(path, window.location.origin).toString()
}

function setStatus(message, isError = false) {
    const el = document.getElementById("status")
    el.textContent = message
    el.style.borderColor = isError ? "#b42318" : "#b9c3cf"
    el.style.background = isError ? "#fff1f0" : "#ffffff"
}

function setBusy(busy) {
    document.getElementById("refresh_params").disabled = busy
    document.getElementById("write_params").disabled = busy || editedValues.size === 0
    document.getElementById("load_param_file").disabled = busy
}

function formatParamValue(value) {
    if (typeof param_to_string === "function") {
        try {
            return param_to_string(Number(value))
        } catch (_e) {
            return String(value)
        }
    }
    return String(value)
}

let paramDocs = null

function escapeHtml(str) {
    if (!str) return ""
    return String(str)
        .replace(/&/g, "&amp;")
        .replace(/</g, "&lt;")
        .replace(/>/g, "&gt;")
        .replace(/"/g, "&quot;")
        .replace(/'/g, "&#039;")
}

async function loadParamDocs() {
    if (paramDocs !== null) return paramDocs
    try {
        const response = await fetch("/webtools/Libraries/arducopter_param_definitions.json")
        if (response.ok) {
            paramDocs = await response.json()
        } else {
            paramDocs = {}
        }
    } catch (e) {
        console.warn("Could not load ArduCopter param documentation:", e)
        paramDocs = {}
    }
    return paramDocs
}

function paramSort(a, b) {
    return a.localeCompare(b, undefined, { numeric: true })
}

function rowsFromParams(params) {
    const docs = paramDocs || {}
    return Object.keys(params).sort(paramSort).map((name) => {
        const doc = docs[name] || {}
        return {
            name,
            displayName: doc.n || "",
            description: doc.d || "",
            units: doc.u || "",
            range: doc.r ? `${doc.r.low} to ${doc.r.high}` : "",
            options: doc.v || null,
            value: Number(params[name].value),
            edited: editedValues.has(name) ? editedValues.get(name) : formatParamValue(params[name].value),
            type: Number(params[name].type || 4),
            changed: editedValues.has(name),
        }
    })
}

function updateSummary() {
    const count = Object.keys(currentParams).length
    const changed = editedValues.size
    document.getElementById("summary").textContent = `${count} params loaded | ${changed} changed`
    document.getElementById("write_params").disabled = changed === 0
}

function buildTable() {
    table = new Tabulator("#table", {
        height: "calc(100vh - 160px)",
        data: rowsFromParams(currentParams),
        index: "name",
        layout: "fitColumns",
        reactiveData: false,
        columns: [
            { title: "Name", field: "name", width: 220, headerSort: true },
            {
                title: "Current",
                field: "value",
                width: 140,
                formatter: (cell) => formatParamValue(cell.getValue()),
            },
            {
                title: "New Value",
                field: "edited",
                width: 140,
                editor: "input",
                cellEdited: (cell) => {
                    const row = cell.getRow()
                    const data = row.getData()
                    const raw = String(cell.getValue()).trim()
                    const numeric = Number(raw)
                    if (!Number.isFinite(numeric)) {
                        alert(`${data.name} must be a number`)
                        data.edited = formatParamValue(data.value)
                        editedValues.delete(data.name)
                    } else if (Math.fround(numeric) === Math.fround(Number(data.value))) {
                        data.edited = formatParamValue(data.value)
                        editedValues.delete(data.name)
                    } else {
                        data.edited = raw
                        editedValues.set(data.name, raw)
                    }
                    data.changed = editedValues.has(data.name)
                    row.update(data)
                    updateSummary()
                },
            },
            {
                title: "Units",
                field: "units",
                width: 85,
                formatter: (cell) => cell.getValue() ? `<span style="font-weight:600; color:#475569;">${escapeHtml(cell.getValue())}</span>` : ""
            },
            {
                title: "Description",
                field: "description",
                minWidth: 380,
                formatter: (cell) => {
                    const data = cell.getRow().getData()
                    const desc = data.description
                    const disp = data.displayName
                    const options = data.options
                    const range = data.range

                    let html = ""
                    if (disp) {
                        html += `<div style="font-weight:600; color:#0f172a; margin-bottom:2px;">${escapeHtml(disp)}</div>`
                    }
                    if (desc) {
                        html += `<div style="color:#334155; line-height:1.35; white-space:normal;">${escapeHtml(desc)}</div>`
                    }
                    if (options && typeof options === "object") {
                        const optEntries = Object.entries(options).map(([k, v]) =>
                            `<span style="display:inline-block; margin-right:8px; font-size:11px; color:#475569;"><b style="color:#0f172a;">${escapeHtml(k)}:</b> ${escapeHtml(v)}</span>`
                        ).join("")
                        html += `<div style="margin-top:4px; padding:3px 6px; background:#f1f5f9; border:1px solid #e2e8f0; border-radius:4px; font-size:11px;">${optEntries}</div>`
                    } else if (range) {
                        html += `<div style="margin-top:3px; font-size:11px; color:#64748b;"><b>Range:</b> ${escapeHtml(range)}</div>`
                    }
                    return html || '<span style="color:#94a3b8; font-style:italic;">No description available</span>'
                },
                tooltip: (cell) => {
                    const data = cell.getRow().getData()
                    return data.description || data.displayName || ""
                }
            },
            {
                title: "State",
                field: "changed",
                width: 90,
                formatter: (cell) => cell.getValue() ? "Changed" : "Same",
            },
            { title: "Type", field: "type", width: 70 },
        ],
        rowFormatter: (row) => {
            row.getElement().style.backgroundColor = row.getData().changed ? "#fff8e1" : ""
        },
        initialSort: [{ column: "name", dir: "asc" }],
    })
}

function renderParams(payload) {
    currentParams = payload.params || {}
    editedValues.clear()
    if (table == null) {
        buildTable()
    } else {
        table.replaceData(rowsFromParams(currentParams))
    }
    const loadedAt = payload.loaded_at ? new Date(payload.loaded_at * 1000).toLocaleString() : "now"
    setStatus(`Loaded ${Object.keys(currentParams).length} parameters from CUAV. Last refresh: ${loadedAt}.`)
    updateSummary()
    applyFilter()
}

async function fetchParams(refresh) {
    setBusy(true)
    try {
        const response = await fetch(apiUrl(`/api/params${refresh ? "?refresh=1" : ""}`))
        const payload = await response.json()
        if (!response.ok || payload.error) {
            throw new Error(payload.error || `HTTP ${response.status}`)
        }
        renderParams(payload)
    } catch (error) {
        setStatus(error.message || String(error), true)
    } finally {
        setBusy(false)
    }
}

async function initialLoad() {
    await loadParamDocs()
    fetchParams(false)
}

function refreshParams() {
    fetchParams(true)
}

function applyFilter() {
    if (table == null) {
        return
    }
    const term = document.getElementById("search").value.trim().toUpperCase()
    table.clearFilter()
    if (term.length > 0) {
        table.setFilter((data) => {
            return (data.name && data.name.toUpperCase().includes(term)) ||
                   (data.displayName && data.displayName.toUpperCase().includes(term)) ||
                   (data.description && data.description.toUpperCase().includes(term)) ||
                   (data.units && data.units.toUpperCase().includes(term))
        })
    }
}

function buildEditedChanges() {
    const changes = []
    for (const [name, rawValue] of editedValues.entries()) {
        const current = currentParams[name]
        if (!current) {
            continue
        }
        changes.push({
            name,
            from: Number(current.value),
            to: Number(rawValue),
            type: Number(current.type || 4),
        })
    }
    return changes.sort((a, b) => paramSort(a.name, b.name))
}

function renderChangeTable(changes, options = {}) {
    const limit = options.limit || 160
    const tableEl = document.createElement("table")
    tableEl.className = "compare-table"
    const head = document.createElement("tr")
    ;["Name", "From", "To"].forEach((label) => {
        const th = document.createElement("th")
        th.textContent = label
        head.appendChild(th)
    })
    tableEl.appendChild(head)

    for (const change of changes.slice(0, limit)) {
        const row = document.createElement("tr")
        ;[change.name, formatParamValue(change.from), formatParamValue(change.to)].forEach((value) => {
            const td = document.createElement("td")
            td.textContent = value
            row.appendChild(td)
        })
        tableEl.appendChild(row)
    }
    if (changes.length > limit) {
        const row = document.createElement("tr")
        const td = document.createElement("td")
        td.colSpan = 3
        td.textContent = `${changes.length - limit} more changes hidden to keep the browser responsive.`
        row.appendChild(td)
        tableEl.appendChild(row)
    }
    return tableEl
}

function showConfirmDialog(title, message, changes, onConfirm) {
    pendingWriteChanges = changes
    document.getElementById("dialog_title").textContent = title
    document.getElementById("dialog_message").textContent = message
    const holder = document.getElementById("dialog_changes")
    holder.replaceChildren(renderChangeTable(changes))
    const button = document.getElementById("dialog_confirm")
    button.onclick = onConfirm
    document.getElementById("confirm_dialog").showModal()
}

function closeDialog() {
    document.getElementById("confirm_dialog").close()
}

function confirmEditedParams() {
    const changes = buildEditedChanges()
    if (changes.length === 0) {
        return
    }
    showConfirmDialog(
        "Confirm Parameter Write",
        `${changes.length} edited parameter value(s) will be written to the CUAV.`,
        changes,
        () => writeChanges(changes),
    )
}

async function writeChanges(changes) {
    closeDialog()
    setBusy(true)
    setStatus(`Writing ${changes.length} parameter value(s) to CUAV...`)
    try {
        const response = await fetch(apiUrl("/api/params/write"), {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ changes }),
        })
        const payload = await response.json()
        if (!response.ok || payload.error) {
            throw new Error(payload.error || `HTTP ${response.status}`)
        }
        for (const item of payload.written || []) {
            currentParams[item.name] = { value: Number(item.actual), type: Number(item.type || 4) }
            editedValues.delete(item.name)
        }
        table.replaceData(rowsFromParams(currentParams))
        const unverified = (payload.written || []).filter((item) => item.verified !== true)
        if (unverified.length > 0) {
            setStatus(`${unverified.length} parameter write(s) did not verify against the requested value. Refresh params before continuing.`, true)
        } else {
            setStatus(`Verified ${payload.count || 0} parameter write(s).`)
        }
        updateSummary()
    } catch (error) {
        setStatus(error.message || String(error), true)
    } finally {
        setBusy(false)
    }
}

async function loadParamFile(event) {
    const file = event.target.files && event.target.files[0]
    event.target.value = ""
    if (!file) {
        return
    }
    setBusy(true)
    try {
        const text = await file.text()
        const response = await fetch(apiUrl("/api/params/compare"), {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ text }),
        })
        const payload = await response.json()
        if (!response.ok || payload.error) {
            throw new Error(payload.error || `HTTP ${response.status}`)
        }
        const changes = payload.changes || []
        if ((payload.missing || []).length > 0) {
            alert(`${payload.missing.length} parameter(s) in the file are not present on the CUAV and will be skipped.`)
        }
        if (changes.length === 0) {
            setStatus(`Loaded ${file.name}: no CUAV parameter changes found.`)
            return
        }
        showConfirmDialog(
            `Confirm ${file.name}`,
            `${changes.length} changed value(s), ${payload.unchanged || 0} unchanged value(s). Confirm to write changed values only.`,
            changes,
            () => writeChanges(changes),
        )
    } catch (error) {
        setStatus(error.message || String(error), true)
    } finally {
        setBusy(false)
    }
}
