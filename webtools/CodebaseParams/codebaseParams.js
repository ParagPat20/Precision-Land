let table = null;
let catalog = {};
let categories = [];
let editedValues = new Map(); // param_name -> new_value
let selectedCategory = "all";

function apiUrl(path) {
    return new URL(path, window.location.origin).toString();
}

function setStatus(message, isError = false) {
    const statusText = document.getElementById("status_text");
    const statusEl = document.getElementById("status");
    if (statusText) statusText.textContent = message;
    if (statusEl) {
        statusEl.style.borderColor = isError ? "#f87171" : "#d0d7de";
        statusEl.style.backgroundColor = isError ? "#fef2f2" : "#ffffff";
    }
}

function updateButtons() {
    const modifiedCount = editedValues.size;
    const saveBtn = document.getElementById("save_params");
    const discardBtn = document.getElementById("discard_changes");
    const counter = document.getElementById("changes_counter");
    
    if (saveBtn) saveBtn.disabled = modifiedCount === 0;
    if (discardBtn) discardBtn.disabled = modifiedCount === 0;
    if (counter) counter.textContent = modifiedCount;

    const total = Object.keys(catalog).length;
    const summary = document.getElementById("summary_text");
    if (summary) {
        summary.textContent = `${total} Total | ${modifiedCount} Unsaved Changes`;
    }
}

function formatDisplayValue(val) {
    if (val === null || val === undefined) return "";
    if (typeof val === "boolean") return val ? "true" : "false";
    return String(val);
}

function castValue(raw, type, choices = null) {
    if (type === "bool") {
        if (typeof raw === "boolean") return raw;
        const str = String(raw).trim().toLowerCase();
        return str === "true" || str === "1" || str === "yes" || str === "on";
    }
    if (type === "int") {
        const num = parseInt(raw, 10);
        return isNaN(num) ? 0 : num;
    }
    if (type === "float") {
        const num = parseFloat(raw);
        return isNaN(num) ? 0.0 : num;
    }
    return String(raw).trim();
}

function rowsFromCatalog() {
    return Object.keys(catalog).map((name) => {
        const p = catalog[name];
        const isModified = editedValues.has(name);
        const currentValue = isModified ? editedValues.get(name) : p.value;
        const isCustom = p.is_custom;

        return {
            name: p.name,
            category: p.category,
            type: p.type,
            default: p.default,
            value: currentValue,
            original_value: p.value,
            unit: p.unit || "-",
            description: p.description || "",
            choices: p.choices || null,
            env_var: p.env_var || "",
            is_modified: isModified,
            is_custom: isCustom
        };
    });
}

function buildCategoriesBar() {
    const container = document.getElementById("categories_container");
    if (!container) return;
    
    container.innerHTML = "";
    
    const allBtn = document.createElement("button");
    allBtn.type = "button";
    allBtn.className = `cat-pill ${selectedCategory === "all" ? "active" : ""}`;
    allBtn.textContent = `All Categories (${Object.keys(catalog).length})`;
    allBtn.onclick = () => filterByCategory("all");
    container.appendChild(allBtn);

    categories.forEach(cat => {
        const catCount = Object.values(catalog).filter(p => p.category === cat).length;
        const btn = document.createElement("button");
        btn.type = "button";
        btn.className = `cat-pill ${selectedCategory === cat ? "active" : ""}`;
        btn.textContent = `${cat} (${catCount})`;
        btn.onclick = () => filterByCategory(cat);
        container.appendChild(btn);
    });
}

function filterByCategory(cat) {
    selectedCategory = cat;
    buildCategoriesBar();
    applyTableFilter();
}

function applyTableFilter() {
    if (!table) return;

    const searchTerm = (document.getElementById("search_input")?.value || "").toLowerCase().trim();
    const stateFilter = document.getElementById("state_filter")?.value || "all";

    table.setFilter((row) => {
        const d = row;
        
        // Category check
        if (selectedCategory !== "all" && d.category !== selectedCategory) {
            return false;
        }

        // State check
        if (stateFilter === "modified" && !d.is_modified) {
            return false;
        }
        if (stateFilter === "custom" && !d.is_custom) {
            return false;
        }

        // Text search
        if (searchTerm) {
            const matchName = d.name.toLowerCase().includes(searchTerm);
            const matchDesc = d.description.toLowerCase().includes(searchTerm);
            const matchCat = d.category.toLowerCase().includes(searchTerm);
            const matchEnv = d.env_var.toLowerCase().includes(searchTerm);
            const matchUnit = d.unit.toLowerCase().includes(searchTerm);
            if (!matchName && !matchDesc && !matchCat && !matchEnv && !matchUnit) {
                return false;
            }
        }

        return true;
    });
}

function buildTable() {
    table = new Tabulator("#table", {
        height: "calc(100vh - 230px)",
        data: rowsFromCatalog(),
        index: "name",
        layout: "fitColumns",
        reactiveData: false,
        rowFormatter: (row) => {
            const data = row.getData();
            if (data.is_modified) {
                row.getElement().classList.add("row-modified");
            } else if (data.is_custom) {
                row.getElement().classList.add("row-customized");
            } else {
                row.getElement().classList.remove("row-modified");
                row.getElement().classList.remove("row-customized");
            }
        },
        columns: [
            {
                title: "Parameter",
                field: "name",
                width: 250,
                headerSort: true,
                formatter: (cell) => {
                    const row = cell.getRow().getData();
                    let html = `<strong>${cell.getValue()}</strong>`;
                    if (row.is_modified) {
                        html += ` <span class="param-badge-modified">MODIFIED</span>`;
                    } else if (row.is_custom) {
                        html += ` <span class="param-badge-custom">CUSTOM</span>`;
                    }
                    return html;
                }
            },
            {
                title: "Category",
                field: "category",
                width: 170,
                headerSort: true,
                formatter: (cell) => `<span class="badge">${cell.getValue()}</span>`
            },
            {
                title: "Value (Click to Edit)",
                field: "value",
                width: 220,
                cssClass: "cell-editable",
                editor: (cell, onRendered, success, cancel, editorParams) => {
                    const row = cell.getRow().getData();
                    let input;

                    if (row.type === "bool") {
                        input = document.createElement("select");
                        input.style.width = "100%";
                        input.style.padding = "4px";
                        const optTrue = document.createElement("option");
                        optTrue.value = "true";
                        optTrue.text = "true";
                        const optFalse = document.createElement("option");
                        optFalse.value = "false";
                        optFalse.text = "false";
                        input.appendChild(optTrue);
                        input.appendChild(optFalse);
                        input.value = row.value ? "true" : "false";

                        input.onchange = () => success(input.value === "true");
                        input.onblur = () => success(input.value === "true");
                    } else if (row.choices && Array.isArray(row.choices)) {
                        input = document.createElement("select");
                        input.style.width = "100%";
                        input.style.padding = "4px";
                        row.choices.forEach(ch => {
                            const opt = document.createElement("option");
                            opt.value = ch;
                            opt.text = String(ch);
                            input.appendChild(opt);
                        });
                        input.value = row.value;

                        input.onchange = () => success(castValue(input.value, row.type));
                        input.onblur = () => success(castValue(input.value, row.type));
                    } else {
                        input = document.createElement("input");
                        input.type = row.type === "int" || row.type === "float" ? "number" : "text";
                        if (row.type === "float") input.step = "any";
                        input.style.width = "100%";
                        input.style.padding = "4px";
                        input.value = row.value;

                        input.onkeydown = (e) => {
                            if (e.key === "Enter") success(castValue(input.value, row.type));
                            if (e.key === "Escape") cancel();
                        };
                        input.onblur = () => success(castValue(input.value, row.type));
                    }

                    onRendered(() => {
                        input.focus();
                        if (input.select) input.select();
                    });

                    return input;
                },
                cellEdited: (cell) => {
                    const row = cell.getRow();
                    const data = row.getData();
                    const original = data.original_value;
                    const newVal = cell.getValue();

                    if (newVal === original) {
                        editedValues.delete(data.name);
                        data.is_modified = false;
                    } else {
                        editedValues.set(data.name, newVal);
                        data.is_modified = true;
                    }
                    data.value = newVal;
                    row.update(data);
                    updateButtons();
                },
                formatter: (cell) => {
                    const row = cell.getRow().getData();
                    return formatDisplayValue(cell.getValue());
                }
            },
            {
                title: "Default",
                field: "default",
                width: 140,
                formatter: (cell) => `<span style="color:var(--text-muted);">${formatDisplayValue(cell.getValue())}</span>`
            },
            {
                title: "Unit",
                field: "unit",
                width: 90,
                formatter: (cell) => `<span style="color:var(--text-muted);">${cell.getValue() || "-"}</span>`
            },
            {
                title: "Type",
                field: "type",
                width: 80,
                formatter: (cell) => `<span class="param-badge-type">${cell.getValue()}</span>`
            },
            {
                title: "Description",
                field: "description",
                formatter: (cell) => {
                    const row = cell.getRow().getData();
                    let text = cell.getValue();
                    if (row.env_var) {
                        text += ` <span style="font-size:11px; color:#64748b; font-family:monospace;">(ENV: ${row.env_var})</span>`;
                    }
                    return text;
                }
            }
        ]
    });
}

async function loadCodebaseParams() {
    setStatus("Fetching codebase parameters from backend...");
    try {
        const res = await fetch(apiUrl("/api/codebase-params"));
        if (!res.ok) {
            throw new Error(`HTTP ${res.status}: ${res.statusText}`);
        }
        const data = await res.json();
        if (data.error) throw new Error(data.error);

        let rawParams = data.params || [];
        catalog = {};
        if (Array.isArray(rawParams)) {
            rawParams.forEach(p => { catalog[p.name] = p; });
        } else if (typeof rawParams === "object") {
            catalog = rawParams;
        }
        categories = data.categories || [];
        editedValues.clear();

        document.getElementById("total_params_badge").textContent = `${Object.keys(catalog).length} Parameters`;
        buildCategoriesBar();

        if (!table) {
            buildTable();
        } else {
            table.setData(rowsFromCatalog());
        }

        applyTableFilter();
        updateButtons();
        setStatus(`Successfully loaded ${Object.keys(catalog).length} parameters across ${categories.length} categories.`);
    } catch (err) {
        console.error(err);
        setStatus(`Failed to load parameters: ${err.message}`, true);
    }
}

function refreshParams() {
    if (editedValues.size > 0) {
        if (!confirm("You have unsaved changes. Discard and reload from server?")) {
            return;
        }
    }
    loadCodebaseParams();
}

function discardChanges() {
    if (editedValues.size === 0) return;
    if (confirm(`Discard all ${editedValues.size} unsaved changes?`)) {
        editedValues.clear();
        table.setData(rowsFromCatalog());
        applyTableFilter();
        updateButtons();
        setStatus("Unsaved edits discarded.");
    }
}

function confirmSaveParams() {
    if (editedValues.size === 0) return;

    const dialog = document.getElementById("confirm_dialog");
    const title = document.getElementById("dialog_title");
    const msg = document.getElementById("dialog_message");
    const content = document.getElementById("dialog_content");
    const confirmBtn = document.getElementById("dialog_confirm_btn");

    title.textContent = `Save ${editedValues.size} Parameter Change(s)`;
    msg.textContent = "The following parameters will be written to config/codebase_params.json and live system environment:";

    let html = `<table class="diff-table">
        <thead>
            <tr><th>Parameter</th><th>Old Value</th><th>New Value</th></tr>
        </thead>
        <tbody>`;

    for (const [name, newVal] of editedValues.entries()) {
        const oldVal = catalog[name] ? catalog[name].value : "-";
        html += `<tr>
            <td><strong>${name}</strong></td>
            <td><span class="diff-old">${formatDisplayValue(oldVal)}</span></td>
            <td><span class="diff-new">${formatDisplayValue(newVal)}</span></td>
        </tr>`;
    }
    html += `</tbody></table>`;
    content.innerHTML = html;

    confirmBtn.className = "primary";
    confirmBtn.textContent = "Confirm & Save";
    confirmBtn.onclick = () => {
        closeDialog();
        saveParamsToServer();
    };

    dialog.showModal();
}

async function saveParamsToServer() {
    const updates = {};
    for (const [name, val] of editedValues.entries()) {
        updates[name] = val;
    }

    setStatus(`Saving ${Object.keys(updates).length} parameter(s) to server...`);
    try {
        const res = await fetch(apiUrl("/api/codebase-params"), {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ updates })
        });

        if (!res.ok) throw new Error(`HTTP ${res.status}: ${res.statusText}`);
        const data = await res.json();
        if (data.error) throw new Error(data.error);

        let rawParams = data.params || [];
        catalog = {};
        if (Array.isArray(rawParams)) {
            rawParams.forEach(p => { catalog[p.name] = p; });
        } else if (typeof rawParams === "object") {
            catalog = rawParams;
        }
        editedValues.clear();

        table.setData(rowsFromCatalog());
        applyTableFilter();
        updateButtons();
        setStatus(`Successfully saved ${data.updated_count} parameter(s) to config/codebase_params.json and environment.`);
    } catch (err) {
        console.error(err);
        setStatus(`Save failed: ${err.message}`, true);
    }
}

function confirmResetDefaults() {
    const dialog = document.getElementById("confirm_dialog");
    const title = document.getElementById("dialog_title");
    const msg = document.getElementById("dialog_message");
    const content = document.getElementById("dialog_content");
    const confirmBtn = document.getElementById("dialog_confirm_btn");

    title.textContent = "Reset Codebase Parameters to Factory Defaults";
    msg.textContent = "Warning: This will clear all overrides in config/codebase_params.json and revert all parameters back to default code values.";
    content.innerHTML = `<p style="color:#b91c1c; font-weight:600;">Are you sure you want to reset all parameters to default?</p>`;

    confirmBtn.className = "danger";
    confirmBtn.textContent = "Reset All to Defaults";
    confirmBtn.onclick = async () => {
        closeDialog();
        setStatus("Resetting parameters to defaults...");
        try {
            const res = await fetch(apiUrl("/api/codebase-params/reset"), {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({})
            });

            if (!res.ok) throw new Error(`HTTP ${res.status}: ${res.statusText}`);
            const data = await res.json();
            if (data.error) throw new Error(data.error);

            let rawParams = data.params || [];
            catalog = {};
            if (Array.isArray(rawParams)) {
                rawParams.forEach(p => { catalog[p.name] = p; });
            } else if (typeof rawParams === "object") {
                catalog = rawParams;
            }
            editedValues.clear();

            table.setData(rowsFromCatalog());
            applyTableFilter();
            updateButtons();
            setStatus(`Successfully reset ${data.reset_count} parameter(s) to factory defaults.`);
        } catch (err) {
            console.error(err);
            setStatus(`Reset failed: ${err.message}`, true);
        }
    };

    dialog.showModal();
}

function closeDialog() {
    const dialog = document.getElementById("confirm_dialog");
    if (dialog) dialog.close();
}

function exportParamsJson() {
    const exportObj = {};
    Object.keys(catalog).forEach(name => {
        exportObj[name] = catalog[name].value;
    });

    const blob = new Blob([JSON.stringify(exportObj, null, 2)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `codebase_params_${new Date().toISOString().slice(0,10)}.json`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
}

function confirmRestartServices() {
    const dialog = document.getElementById("confirm_dialog");
    const title = document.getElementById("dialog_title");
    const msg = document.getElementById("dialog_message");
    const content = document.getElementById("dialog_content");
    const confirmBtn = document.getElementById("dialog_confirm_btn");

    title.textContent = "⚡ Full System / Service Restart";
    msg.textContent = "This will restart all companion services (Precision-Land, WebTools, RTSP Streamer, and Servo Controller).";
    content.innerHTML = `<div style="background:#fee2e2; border:1px solid #f87171; padding:12px; border-radius:6px; color:#991b1b; font-size:13px;">
        <strong>Warning:</strong> The web connection will momentarily disconnect while processes reload (~3-5 seconds).
    </div>`;

    confirmBtn.className = "danger";
    confirmBtn.textContent = "Restart All Services";
    confirmBtn.onclick = async () => {
        closeDialog();
        setStatus("Sending restart signal to companion computer...");
        try {
            const res = await fetch(apiUrl("/api/system/restart"), {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({})
            });
            const data = await res.json();
            setStatus(data.message || "Restart signal sent. Reconnecting in 5 seconds...");
            setTimeout(() => {
                window.location.reload();
            }, 5000);
        } catch (err) {
            // Expected if server goes down immediately
            setStatus("Restart in progress... Reconnecting page in 5 seconds...");
            setTimeout(() => {
                window.location.reload();
            }, 5000);
        }
    };

    dialog.showModal();
}

function confirmGitPull() {
    const dialog = document.getElementById("confirm_dialog");
    const title = document.getElementById("dialog_title");
    const msg = document.getElementById("dialog_message");
    const content = document.getElementById("dialog_content");
    const confirmBtn = document.getElementById("dialog_confirm_btn");

    title.textContent = "☁ Pull Latest Git Updates";
    msg.textContent = "Run 'git pull' on the companion computer repository to fetch and merge latest commits from remote:";
    content.innerHTML = `<p style="color:var(--text-muted); font-size:13px;">This will pull newest code updates onto the drone. Uncommitted parameter overrides in config/codebase_params.json are preserved.</p>`;

    confirmBtn.className = "primary";
    confirmBtn.textContent = "Pull Updates (git pull)";
    confirmBtn.onclick = async () => {
        closeDialog();
        setStatus("Executing 'git pull' on companion repository...");
        try {
            const res = await fetch(apiUrl("/api/system/git-pull"), {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({})
            });
            const data = await res.json();
            if (data.ok) {
                setStatus(`Git pull succeeded: ${data.output}`);
                alert(`Git Pull Result:\n\n${data.output}`);
            } else {
                setStatus(`Git pull warning/error: ${data.output}`, true);
                alert(`Git Pull Output:\n\n${data.output}`);
            }
        } catch (err) {
            console.error(err);
            setStatus(`Git pull request failed: ${err.message}`, true);
        }
    };

    dialog.showModal();
}

function initCodebaseParams() {
    loadCodebaseParams();
}
