let import_done = [];
let DataflashParser;
import_done[0] = import("../modules/JsDataflashParser/parser.js").then((mod) => {
    DataflashParser = mod.default || mod.DataflashParser || mod;
});
import "https://cdn.jsdelivr.net/npm/marked@4.3.0/marked.min.js";
import "https://cdn.jsdelivr.net/npm/dompurify@3.0.5/dist/purify.min.js";
import { OpenAI } from "https://cdn.jsdelivr.net/npm/openai@4.85.4/+esm";

const SETTINGS_STORAGE_KEY = "aiLogAnalyzer.settings";
const DEFAULT_LOCAL_BASE_URL = "http://127.0.0.1:11434/v1";
const DEFAULT_LOCAL_MODEL = "qwen3.5";
const DEFAULT_GEMINI_BASE_URL = "https://generativelanguage.googleapis.com/v1beta/openai/";
const DEFAULT_GEMINI_MODEL = "gemini-2.5-flash";
const DEFAULT_OPENAI_BASE_URL = "https://api.openai.com/v1";
const DEFAULT_OPENAI_MODEL = "gpt-4.1-mini";
const MAX_TOOL_LOOPS = 4;
const MAX_SAMPLE_ROWS = 20;
const MAX_NUMERIC_FIELDS = 12;

let client = null;
let aiSettings = null;
let isProcessing = false;
let log = null;
let currentLogFileName = null;
let cachedMessageData = new Map();
let conversationHistory = [];

let chatMessages;
let fileInput;
let messageInput;
let sendButton;
let fileUploadLabel;
let vizArea;
let activeStreamingMessage = null;

function getDefaultSettings() {
    return {
        provider: "local",
        localBaseUrl: DEFAULT_LOCAL_BASE_URL,
        localModel: DEFAULT_LOCAL_MODEL,
        localApiKey: "",
        localDisableThinking: true,
        geminiApiKey: "",
        geminiModel: DEFAULT_GEMINI_MODEL,
        openaiApiKey: "",
        openaiModel: DEFAULT_OPENAI_MODEL
    };
}

function loadSettings() {
    try {
        const parsed = JSON.parse(localStorage.getItem(SETTINGS_STORAGE_KEY) || "null");
        return { ...getDefaultSettings(), ...(parsed || {}) };
    } catch (error) {
        console.warn("Failed to load saved AI settings:", error);
        return getDefaultSettings();
    }
}

function saveSettings(settings) {
    aiSettings = { ...getDefaultSettings(), ...settings };
    localStorage.setItem(SETTINGS_STORAGE_KEY, JSON.stringify(aiSettings));
    client = null;
}

function hasUsableSettings(settings) {
    if (!settings) {
        return false;
    }

    if (settings.provider === "gemini") {
        return Boolean(settings.geminiApiKey && settings.geminiModel);
    }

    if (settings.provider === "openai") {
        return Boolean(settings.openaiApiKey && settings.openaiModel);
    }

    return Boolean(settings.localBaseUrl && settings.localModel);
}

function getClientConfig() {
    if (!aiSettings) {
        aiSettings = loadSettings();
    }

    if (aiSettings.provider === "gemini") {
        return {
            provider: "gemini",
            apiKey: aiSettings.geminiApiKey,
            baseURL: DEFAULT_GEMINI_BASE_URL,
            model: aiSettings.geminiModel
        };
    }

    if (aiSettings.provider === "openai") {
        return {
            provider: "openai",
            apiKey: aiSettings.openaiApiKey,
            baseURL: DEFAULT_OPENAI_BASE_URL,
            model: aiSettings.openaiModel
        };
    }

    return {
        provider: "local",
        apiKey: aiSettings.localApiKey || "ollama",
        baseURL: aiSettings.localBaseUrl,
        model: aiSettings.localModel
    };
}

function providerLabel(provider) {
    if (provider === "gemini") {
        return "Gemini";
    }
    if (provider === "openai") {
        return "ChatGPT / OpenAI";
    }
    return "local model";
}

function escapeHtml(value) {
    return String(value)
        .replaceAll("&", "&amp;")
        .replaceAll("<", "&lt;")
        .replaceAll(">", "&gt;")
        .replaceAll('"', "&quot;")
        .replaceAll("'", "&#39;");
}

function createModal(title, content) {
    const modalContainer = document.createElement("div");
    modalContainer.className = "modal";

    const modalContent = document.createElement("div");
    modalContent.className = "modal-content";

    const modalTitle = document.createElement("h2");
    modalTitle.textContent = title;

    const closeButton = document.createElement("span");
    closeButton.className = "close-modal";
    closeButton.innerHTML = "&times;";
    closeButton.onclick = () => modalContainer.remove();

    const modalBody = document.createElement("div");
    modalBody.className = "modal-body";
    modalBody.innerHTML = content;

    modalContent.appendChild(closeButton);
    modalContent.appendChild(modalTitle);
    modalContent.appendChild(modalBody);
    modalContainer.appendChild(modalContent);

    return modalContainer;
}

function showSettingsPrompt() {
    const settings = aiSettings || loadSettings();
    const modal = createModal(
        "AI Settings",
        `
        <p>Choose which backend the log analyzer should use.</p>
        <form id="aiSettingsForm">
            <label for="providerSelect">Provider</label>
            <select id="providerSelect" class="settings-input">
                <option value="local" ${settings.provider === "local" ? "selected" : ""}>Local model</option>
                <option value="gemini" ${settings.provider === "gemini" ? "selected" : ""}>Gemini</option>
                <option value="openai" ${settings.provider === "openai" ? "selected" : ""}>ChatGPT / OpenAI</option>
            </select>

            <div id="localSettings">
                <label for="localBaseUrlInput">Local OpenAI-compatible base URL</label>
                <input class="settings-input" type="text" id="localBaseUrlInput" value="${escapeHtml(settings.localBaseUrl || DEFAULT_LOCAL_BASE_URL)}" placeholder="${DEFAULT_LOCAL_BASE_URL}">

                <label for="localModelInput">Local model name</label>
                <input class="settings-input" type="text" id="localModelInput" value="${escapeHtml(settings.localModel || DEFAULT_LOCAL_MODEL)}" placeholder="${DEFAULT_LOCAL_MODEL}">
                <datalist id="localModelOptions"></datalist>
                <button type="button" id="fetchLocalModelsBtn" class="secondary-btn">Fetch installed local models</button>

                <label for="localApiKeyInput">Local API key (optional)</label>
                <input class="settings-input" type="password" id="localApiKeyInput" value="${escapeHtml(settings.localApiKey || "")}" placeholder="Optional">

                <label class="checkbox-row">
                    <input type="checkbox" id="localDisableThinkingInput" ${settings.localDisableThinking ? "checked" : ""}>
                    <span>Disable thinking for local models when supported</span>
                </label>
            </div>

            <div id="geminiSettings">
                <label for="geminiApiKeyInput">Gemini API key</label>
                <input class="settings-input" type="password" id="geminiApiKeyInput" value="${escapeHtml(settings.geminiApiKey || "")}" placeholder="AIza...">

                <label for="geminiModelInput">Gemini model</label>
                <input class="settings-input" type="text" id="geminiModelInput" value="${escapeHtml(settings.geminiModel || DEFAULT_GEMINI_MODEL)}" placeholder="${DEFAULT_GEMINI_MODEL}">
            </div>

            <div id="openaiSettings">
                <label for="openaiApiKeyInput">OpenAI API key</label>
                <input class="settings-input" type="password" id="openaiApiKeyInput" value="${escapeHtml(settings.openaiApiKey || "")}" placeholder="sk-...">

                <label for="openaiModelInput">OpenAI model</label>
                <input class="settings-input" type="text" id="openaiModelInput" value="${escapeHtml(settings.openaiModel || DEFAULT_OPENAI_MODEL)}" placeholder="${DEFAULT_OPENAI_MODEL}">
            </div>

            <button type="submit" class="submit-btn">Save</button>
        </form>
        `
    );

    document.body.appendChild(modal);

    const providerSelect = modal.querySelector("#providerSelect");
    const localSettings = modal.querySelector("#localSettings");
    const geminiSettings = modal.querySelector("#geminiSettings");
    const openaiSettings = modal.querySelector("#openaiSettings");
    const localBaseUrlInput = modal.querySelector("#localBaseUrlInput");
    const localModelInput = modal.querySelector("#localModelInput");
    const localModelOptions = modal.querySelector("#localModelOptions");
    const fetchLocalModelsBtn = modal.querySelector("#fetchLocalModelsBtn");
    localModelInput.setAttribute("list", "localModelOptions");

    const syncProviderSections = () => {
        const provider = providerSelect.value;
        localSettings.style.display = provider === "local" ? "block" : "none";
        geminiSettings.style.display = provider === "gemini" ? "block" : "none";
        openaiSettings.style.display = provider === "openai" ? "block" : "none";
    };

    const fetchLocalModels = async () => {
        fetchLocalModelsBtn.disabled = true;
        fetchLocalModelsBtn.textContent = "Loading...";
        try {
            const baseUrl = localBaseUrlInput.value.trim() || DEFAULT_LOCAL_BASE_URL;
            const url = new URL("/api/tags", baseUrl).toString();
            const response = await fetch(url);
            if (!response.ok) {
                throw new Error(`HTTP ${response.status}`);
            }

            const data = await response.json();
            const names = Array.isArray(data.models) ? data.models.map((model) => model.name).filter(Boolean) : [];
            localModelOptions.innerHTML = "";
            names.forEach((name) => {
                const option = document.createElement("option");
                option.value = name;
                localModelOptions.appendChild(option);
            });

            if (names.length > 0) {
                if (!names.includes(localModelInput.value.trim())) {
                    localModelInput.value = names[0];
                }
                addChatMessage(`Loaded ${names.length} local model${names.length === 1 ? "" : "s"} from Ollama.`, "system");
            } else {
                addChatMessage("No local models were returned by the Ollama server.", "error");
            }
        } catch (error) {
            addChatMessage(`Failed to fetch local models: ${error.message}`, "error");
        } finally {
            fetchLocalModelsBtn.disabled = false;
            fetchLocalModelsBtn.textContent = "Fetch installed local models";
        }
    };

    providerSelect.addEventListener("change", syncProviderSections);
    fetchLocalModelsBtn.addEventListener("click", fetchLocalModels);
    syncProviderSections();

    modal.querySelector("#aiSettingsForm").addEventListener("submit", async (event) => {
        event.preventDefault();

        const nextSettings = {
            provider: providerSelect.value,
            localBaseUrl: modal.querySelector("#localBaseUrlInput").value.trim(),
            localModel: modal.querySelector("#localModelInput").value.trim(),
            localApiKey: modal.querySelector("#localApiKeyInput").value.trim(),
            localDisableThinking: modal.querySelector("#localDisableThinkingInput").checked,
            geminiApiKey: modal.querySelector("#geminiApiKeyInput").value.trim(),
            geminiModel: modal.querySelector("#geminiModelInput").value.trim(),
            openaiApiKey: modal.querySelector("#openaiApiKeyInput").value.trim(),
            openaiModel: modal.querySelector("#openaiModelInput").value.trim()
        };

        if (nextSettings.provider === "local" && (!nextSettings.localBaseUrl || !nextSettings.localModel)) {
            addChatMessage("Local mode requires a base URL and model name.", "error");
            return;
        }

        if (nextSettings.provider === "gemini" && (!nextSettings.geminiApiKey || !nextSettings.geminiModel)) {
            addChatMessage("Gemini mode requires an API key and model.", "error");
            return;
        }

        if (nextSettings.provider === "openai" && (!nextSettings.openaiApiKey || !nextSettings.openaiModel)) {
            addChatMessage("OpenAI mode requires an API key and model.", "error");
            return;
        }

        saveSettings(nextSettings);
        modal.remove();

        try {
            await connectIfNeeded();
            addChatMessage(`Using ${providerLabel(aiSettings.provider)} for analysis.`, "system");
        } catch (error) {
            addChatMessage(`AI settings saved, but connection failed: ${error.message}`, "error");
        }
    });
}

async function connectIfNeeded() {
    if (!hasUsableSettings(aiSettings || loadSettings())) {
        showSettingsPrompt();
        throw new Error("AI settings are required");
    }

    if (client) {
        return client;
    }

    const config = getClientConfig();
    client = new OpenAI({
        apiKey: config.apiKey,
        baseURL: config.baseURL,
        dangerouslyAllowBrowser: true
    });

    return client;
}

async function initializeApp() {
    aiSettings = loadSettings();

    if (!hasUsableSettings(aiSettings)) {
        showSettingsPrompt();
        return;
    }

    try {
        await connectIfNeeded();
        addChatMessage(`Configured for ${providerLabel(aiSettings.provider)}. Upload a log file or ask a question.`, "system");
    } catch (error) {
        console.error("Failed to initialize AI client:", error);
        addChatMessage("AI connection is not ready. Open AI Settings and verify the provider details.", "error");
    }
}

function showThinkingMessage(show) {
    const existingIndicator = document.getElementById("thinking-message");

    if (show && chatMessages) {
        if (existingIndicator) {
            return;
        }

        const thinkingIndicator = document.createElement("div");
        thinkingIndicator.id = "thinking-message";
        thinkingIndicator.className = "message thinking-indicator";

        const dotPulse = document.createElement("div");
        dotPulse.className = "dot-pulse";
        dotPulse.textContent = "...";

        const dotsAnimation = setInterval(() => {
            if (dotPulse.textContent === ".") {
                dotPulse.textContent = "..";
            } else if (dotPulse.textContent === "..") {
                dotPulse.textContent = "...";
            } else {
                dotPulse.textContent = ".";
            }
        }, 500);

        thinkingIndicator.dataset.intervalId = dotsAnimation;
        thinkingIndicator.appendChild(dotPulse);
        chatMessages.appendChild(thinkingIndicator);
        chatMessages.scrollTop = chatMessages.scrollHeight;
    } else if (!show && existingIndicator) {
        if (existingIndicator.dataset.intervalId) {
            clearInterval(parseInt(existingIndicator.dataset.intervalId, 10));
        }
        existingIndicator.remove();
    }
}

function setProcessingState(isActive) {
    isProcessing = isActive;

    if (sendButton) {
        sendButton.disabled = isActive;
    }

    if (messageInput) {
        messageInput.disabled = isActive;
        messageInput.placeholder = isActive ? "Assistant is working..." : "Ask about your flight data...";
    }

    if (fileUploadLabel) {
        fileUploadLabel.classList.toggle("processing", isActive);
    }
}

let assistantBuffer = "";
function addChatMessage(content, sender) {
    const messagesContainer = document.getElementById("chatMessages");
    if (!messagesContainer) {
        return;
    }

    const messageElement = document.createElement("div");
    messageElement.classList.add("message");

    if (sender === "assistant" || sender === "ai") {
        assistantBuffer = typeof content === "string" ? content : "";
        messageElement.classList.add("ai-message", "markdown");
        messageElement.innerHTML = DOMPurify.sanitize(marked.parse(assistantBuffer || ""));
    } else if (sender === "user") {
        messageElement.classList.add("user-message");
        messageElement.textContent = content;
    } else if (sender === "system" || sender === "error") {
        messageElement.classList.add("system-message");
        if (sender === "error") {
            messageElement.classList.add("error-message");
        }
        messageElement.textContent = content;
    } else {
        messageElement.textContent = typeof content === "string" ? content : "Content could not be displayed";
    }

    messagesContainer.appendChild(messageElement);
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
}

function ensureStreamingAssistantMessage() {
    if (activeStreamingMessage && document.body.contains(activeStreamingMessage)) {
        return activeStreamingMessage;
    }

    const messagesContainer = document.getElementById("chatMessages");
    if (!messagesContainer) {
        return null;
    }

    const messageElement = document.createElement("div");
    messageElement.classList.add("message", "ai-message", "markdown");
    messageElement.dataset.streaming = "true";
    messageElement.textContent = "";
    messagesContainer.appendChild(messageElement);
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
    activeStreamingMessage = messageElement;
    return messageElement;
}

function updateStreamingAssistantMessage(content) {
    const messageElement = ensureStreamingAssistantMessage();
    if (!messageElement) {
        return;
    }

    messageElement.textContent = content || "";
    const messagesContainer = document.getElementById("chatMessages");
    if (messagesContainer) {
        messagesContainer.scrollTop = messagesContainer.scrollHeight;
    }
}

function finalizeStreamingAssistantMessage(content) {
    const messageElement = ensureStreamingAssistantMessage();
    if (!messageElement) {
        return;
    }

    assistantBuffer = typeof content === "string" ? content : "";
    messageElement.innerHTML = DOMPurify.sanitize(marked.parse(assistantBuffer || ""));
    delete messageElement.dataset.streaming;
    activeStreamingMessage = null;

    const messagesContainer = document.getElementById("chatMessages");
    if (messagesContainer) {
        messagesContainer.scrollTop = messagesContainer.scrollHeight;
    }
}

function clearStreamingAssistantMessage() {
    if (activeStreamingMessage && document.body.contains(activeStreamingMessage)) {
        activeStreamingMessage.remove();
    }
    activeStreamingMessage = null;
}

async function loadLog(logFile) {
    await Promise.allSettled(import_done);

    if (typeof DataflashParser !== "function") {
        throw new Error("Dataflash parser could not be loaded");
    }

    log = new DataflashParser();
    log.processData(logFile, []);
    cachedMessageData.clear();
    conversationHistory = [];
}

async function handleFileUpload(event) {
    const file = event.target.files[0];
    if (!file) {
        return;
    }

    fileUploadLabel.textContent = `Selected: ${file.name}`;
    fileUploadLabel.classList.add("file-selected");
    addChatMessage(`Processing ${file.name}...`, "system");
    setProcessingState(true);

    try {
        if (!file.name.toLowerCase().endsWith(".bin")) {
            throw new Error("Only .bin log files are supported right now");
        }

        const arrayBuffer = await file.arrayBuffer();
        await loadLog(arrayBuffer);
        currentLogFileName = file.name;

        addChatMessage("Log file uploaded successfully. You can now ask questions about the log.", "system");
        updateVisualization({
            summary: "Log File Ready",
            issues: ["Ask a question to inspect message data or generate a simple plot."]
        });
    } catch (error) {
        console.error("Failed to load log:", error);
        addChatMessage(`Failed to load log file: ${error.message}`, "error");
    } finally {
        setProcessingState(false);
    }
}

function updateVisualization(data) {
    if (!vizArea) {
        return;
    }

    Array.from(vizArea.children).forEach((child) => {
        if (!child.classList.contains("graph-container")) {
            child.remove();
        }
    });

    if (data.summary) {
        const summarySection = document.createElement("div");
        summarySection.className = "viz-section";
        summarySection.id = "summary-section";

        const summaryTitle = document.createElement("h3");
        summaryTitle.textContent = "Summary";
        summarySection.appendChild(summaryTitle);

        const summaryContent = document.createElement("p");
        summaryContent.textContent = data.summary;
        summarySection.appendChild(summaryContent);
        vizArea.insertBefore(summarySection, vizArea.firstChild);
    }

    if (data.issues && data.issues.length > 0) {
        const issuesSection = document.createElement("div");
        issuesSection.className = "viz-section";
        issuesSection.id = "findings-section";

        const issuesTitle = document.createElement("h3");
        issuesTitle.textContent = "Findings";
        issuesSection.appendChild(issuesTitle);

        const issuesList = document.createElement("ul");
        data.issues.forEach((issue) => {
            const issueItem = document.createElement("div");
            issueItem.textContent = issue;
            issuesList.appendChild(issueItem);
        });

        issuesSection.appendChild(issuesList);
        const summarySection = document.getElementById("summary-section");
        if (summarySection) {
            vizArea.insertBefore(issuesSection, summarySection.nextSibling);
        } else {
            vizArea.insertBefore(issuesSection, vizArea.firstChild);
        }
    }
}

function clearGraphs() {
    if (!vizArea) {
        return;
    }
    Array.from(vizArea.querySelectorAll(".graph-container")).forEach((element) => element.remove());
}

function addGraphContainer(title) {
    const placeholder = vizArea.querySelector(".viz-placeholder");
    if (placeholder) {
        placeholder.remove();
    }

    const graphSection = document.createElement("div");
    graphSection.className = "viz-section graph-container";

    const graphTitle = document.createElement("h3");
    graphTitle.textContent = title;
    graphSection.appendChild(graphTitle);

    const plotDiv = document.createElement("div");
    plotDiv.style.width = "100%";
    plotDiv.style.minHeight = "320px";
    graphSection.appendChild(plotDiv);
    vizArea.appendChild(graphSection);

    return plotDiv;
}

function getTimeLikeField(fields) {
    const preferred = ["TimeUS", "time_boot_ms", "TimeMS", "timestamp", "time"];
    return preferred.find((field) => fields.includes(field)) || fields[0];
}

function toFiniteNumber(value) {
    const number = Number(value);
    return Number.isFinite(number) ? number : NaN;
}

function normalizeAxisValues(rows, fieldName) {
    if (!rows.length) {
        return [];
    }

    if (!fieldName) {
        return rows.map((_, index) => index);
    }

    if (fieldName === "TimeUS") {
        return rows.map((row) => toFiniteNumber(row[fieldName]) / 1e6);
    }

    if (fieldName === "time_boot_ms" || fieldName === "TimeMS") {
        return rows.map((row) => toFiniteNumber(row[fieldName]) / 1e3);
    }

    return rows.map((row, index) => row[fieldName] ?? index);
}

function formatAxisLabel(fieldName) {
    if (fieldName === "TimeUS" || fieldName === "time_boot_ms" || fieldName === "TimeMS") {
        return "Time (s)";
    }
    return fieldName;
}

function getMessageData(messageType, instance) {
    const cacheKey = `${messageType}::${instance || ""}`;
    if (cachedMessageData.has(cacheKey)) {
        return cachedMessageData.get(cacheKey);
    }

    if (!log || !log.messageTypes || !(messageType in log.messageTypes)) {
        return null;
    }

    const messageInfo = log.messageTypes[messageType];
    let resolvedInstance = instance ?? null;
    let rows;

    if ("instances" in messageInfo) {
        const instances = Object.keys(messageInfo.instances || {});
        if (!resolvedInstance) {
            resolvedInstance = instances[0] || null;
        }
        if (!resolvedInstance) {
            return null;
        }
        rows = log.get_instance(messageType, resolvedInstance);
    } else {
        rows = log.get(messageType);
    }

    if (!Array.isArray(rows)) {
        return null;
    }

    const fields = rows.length > 0 ? Object.keys(rows[0]) : [];
    const payload = { messageType, instance: resolvedInstance, rows, fields };
    cachedMessageData.set(cacheKey, payload);
    return payload;
}

function sampleRows(rows, maxRows = MAX_SAMPLE_ROWS) {
    if (rows.length <= maxRows) {
        return rows;
    }

    const sampled = [];
    const step = (rows.length - 1) / (maxRows - 1);
    for (let i = 0; i < maxRows; i++) {
        sampled.push(rows[Math.round(i * step)]);
    }
    return sampled;
}

function summarizeNumericField(rows, fieldName) {
    const values = rows.map((row) => toFiniteNumber(row[fieldName])).filter((value) => Number.isFinite(value));
    if (values.length === 0) {
        return null;
    }

    let min = values[0];
    let max = values[0];
    let sum = 0;
    for (const value of values) {
        min = Math.min(min, value);
        max = Math.max(max, value);
        sum += value;
    }

    return {
        min,
        max,
        mean: sum / values.length,
        samples: values.length
    };
}

function buildMessageDataSummary(messageType, instance) {
    const raw = getMessageData(messageType, instance);
    if (!raw) {
        return {
            ok: false,
            error: `Message type ${messageType} was not found in the uploaded log.`
        };
    }

    const numericFields = raw.fields.filter((field) => {
        for (const row of raw.rows.slice(0, 20)) {
            if (Number.isFinite(toFiniteNumber(row[field]))) {
                return true;
            }
        }
        return false;
    }).slice(0, MAX_NUMERIC_FIELDS);

    const numericStats = {};
    numericFields.forEach((field) => {
        const stats = summarizeNumericField(raw.rows, field);
        if (stats) {
            numericStats[field] = stats;
        }
    });

    return {
        ok: true,
        message_type: messageType,
        instance: raw.instance,
        row_count: raw.rows.length,
        fields: raw.fields,
        suggested_time_field: getTimeLikeField(raw.fields),
        numeric_stats: numericStats,
        sample_rows: sampleRows(raw.rows)
    };
}

function listAvailableMessageTypes() {
    if (!log || !log.messageTypes) {
        return {
            ok: false,
            error: "No log file has been uploaded yet."
        };
    }

    return {
        ok: true,
        log_file: currentLogFileName,
        message_types: Object.entries(log.messageTypes).map(([name, info]) => ({
            name,
            instances: "instances" in info ? Object.keys(info.instances || {}) : []
        }))
    };
}

function drawPlotFromRequest(plotRequest) {
    if (!window.Plotly || !plotRequest || !plotRequest.message_type) {
        return false;
    }

    const raw = getMessageData(plotRequest.message_type, plotRequest.instance);
    if (!raw || raw.rows.length === 0) {
        return false;
    }

    const availableFields = raw.fields;
    const xField = availableFields.includes(plotRequest.x_field) ? plotRequest.x_field : getTimeLikeField(availableFields);
    const yFields = Array.isArray(plotRequest.y_fields) ? plotRequest.y_fields.filter((field) => availableFields.includes(field)) : [];

    if (yFields.length === 0) {
        return false;
    }

    clearGraphs();
    const plotDiv = addGraphContainer(plotRequest.title || `${plotRequest.message_type} plot`);
    const x = normalizeAxisValues(raw.rows, xField);

    const traces = yFields.map((field) => ({
        type: "scattergl",
        mode: "lines",
        name: field,
        x,
        y: raw.rows.map((row) => toFiniteNumber(row[field]))
    }));

    Plotly.newPlot(plotDiv, traces, {
        title: { text: plotRequest.title || `${plotRequest.message_type} plot` },
        margin: { b: 48, l: 60, r: 20, t: 48 },
        xaxis: { title: { text: formatAxisLabel(xField) }, zeroline: false, showline: true, mirror: true },
        yaxis: { title: { text: yFields.length === 1 ? yFields[0] : "Value" }, zeroline: false, showline: true, mirror: true },
        legend: { itemclick: false, itemdoubleclick: false }
    }, { displaylogo: false, responsive: true });

    return true;
}

function buildSystemPrompt() {
    const logState = log
        ? `A log file named ${currentLogFileName || "uploaded log"} is loaded.`
        : "No log file is currently loaded.";

    return `
You analyze ArduPilot flight logs.
${logState}

Always reply with a single JSON object and nothing else.

Allowed response shapes:
{
  "action": "list_message_types",
  "reason": "Need to inspect available messages first."
}

{
  "action": "request_log_data",
  "message_type": "GPS",
  "instance": "0",
  "reason": "Need GPS altitude and fix data."
}

{
  "action": "final",
  "markdown": "Short markdown answer for the user.",
  "summary": "Short visualization summary.",
  "issues": ["Finding 1", "Finding 2"],
  "plot_request": {
    "message_type": "GPS",
    "instance": "0",
    "x_field": "TimeUS",
    "y_fields": ["Alt"],
    "title": "GPS altitude"
  }
}

Rules:
- Use list_message_types if the user asks about the log but the relevant message type is unclear.
- Use request_log_data when you need actual data from the uploaded log.
- Keep markdown concise and technical.
- Only include plot_request when a simple line plot is helpful.
- If you already have enough information, always prefer "final" over another tool request.
- If you are unsure about the exact JSON shape, still return an object with action "final".
- Never output plain prose outside the JSON object.
`.trim();
}

function buildChatMessages() {
    return [
        { role: "system", content: buildSystemPrompt() },
        ...conversationHistory
    ];
}

async function requestModelResponse(onProgress) {
    const openai = await connectIfNeeded();
    const config = getClientConfig();
    const request = {
        model: config.model,
        temperature: 0.2,
        messages: buildChatMessages()
    };

    if (config.provider === "local" && aiSettings?.localDisableThinking) {
        request.think = false;
    }

    try {
        const stream = await openai.chat.completions.create({
            ...request,
            stream: true
        });

        let content = "";
        for await (const chunk of stream) {
            const delta = chunk?.choices?.[0]?.delta?.content;
            if (typeof delta === "string" && delta.length > 0) {
                content += delta;
                onProgress?.(content);
            }
        }

        if (!content) {
            throw new Error("Model returned an empty response");
        }
        return content;
    } catch (streamError) {
        const response = await openai.chat.completions.create(request);
        const content = response?.choices?.[0]?.message?.content;
        if (!content) {
            throw new Error(streamError?.message || "Model returned an empty response");
        }
        onProgress?.(content);
        return content;
    }
}

function extractJsonObject(text) {
    const trimmed = text.trim();
    const fenced = trimmed.match(/```(?:json)?\s*([\s\S]*?)```/i);
    const candidate = fenced ? fenced[1].trim() : trimmed;

    try {
        return JSON.parse(candidate);
    } catch (_) {
        const firstBrace = candidate.indexOf("{");
        const lastBrace = candidate.lastIndexOf("}");
        if (firstBrace >= 0 && lastBrace > firstBrace) {
            return JSON.parse(candidate.slice(firstBrace, lastBrace + 1));
        }
        throw new Error("Assistant did not return valid JSON");
    }
}

function normalizeAssistantResponse(rawText) {
    let parsed = null;

    try {
        parsed = extractJsonObject(rawText);
    } catch (_) {
        return {
            action: "final",
            markdown: rawText.trim() || "No answer returned.",
            summary: "Analysis complete.",
            issues: []
        };
    }

    if (!parsed || typeof parsed !== "object") {
        return {
            action: "final",
            markdown: rawText.trim() || "No answer returned.",
            summary: "Analysis complete.",
            issues: []
        };
    }

    if (!parsed.action) {
        if (typeof parsed.markdown === "string") {
            parsed.action = "final";
        } else if (typeof parsed.response === "string") {
            parsed = {
                action: "final",
                markdown: parsed.response,
                summary: "Analysis complete.",
                issues: []
            };
        } else {
            parsed = {
                action: "final",
                markdown: rawText.trim() || "No answer returned.",
                summary: "Analysis complete.",
                issues: []
            };
        }
    }

    if (parsed.action === "final") {
        parsed.markdown = typeof parsed.markdown === "string" && parsed.markdown.trim().length > 0
            ? parsed.markdown
            : (rawText.trim() || "No answer returned.");
        parsed.summary = typeof parsed.summary === "string" && parsed.summary.trim().length > 0
            ? parsed.summary
            : "Analysis complete.";
        parsed.issues = Array.isArray(parsed.issues) ? parsed.issues : [];
        return parsed;
    }

    if ((parsed.action === "request_log_data" || parsed.action === "list_message_types")) {
        return parsed;
    }

    return {
        action: "final",
        markdown: rawText.trim() || "No answer returned.",
        summary: "Analysis complete.",
        issues: []
    };
}

function buildFallbackFinalResponse(rawText, lastStructuredResponse) {
    if (lastStructuredResponse?.action === "final") {
        return normalizeAssistantResponse(JSON.stringify(lastStructuredResponse));
    }

    let markdown = rawText?.trim() || "The local model did not return a structured final answer.";
    if (!markdown) {
        markdown = "The local model did not return a structured final answer.";
    }

    return {
        action: "final",
        markdown,
        summary: "Partial analysis",
        issues: ["The model returned an unstructured or incomplete response, so this answer was recovered automatically."]
    };
}

function appendToolResult(action, payload) {
    conversationHistory.push({
        role: "user",
        content: `TOOL_RESULT ${action}\n${JSON.stringify(payload)}`
    });
}

async function processUserMessage(message) {
    setProcessingState(true);
    showThinkingMessage(true);
    clearStreamingAssistantMessage();

    try {
        await connectIfNeeded();
        conversationHistory.push({ role: "user", content: message });

        let finalPayload = null;
        let lastRawResponse = "";
        let lastStructuredResponse = null;

        for (let i = 0; i < MAX_TOOL_LOOPS; i++) {
            const rawResponse = await requestModelResponse((partialText) => {
                showThinkingMessage(false);
                updateStreamingAssistantMessage(partialText);
            });
            lastRawResponse = rawResponse;
            const response = normalizeAssistantResponse(rawResponse);
            lastStructuredResponse = response;
            conversationHistory.push({ role: "assistant", content: JSON.stringify(response) });

            if (response.action === "list_message_types") {
                appendToolResult("list_message_types", listAvailableMessageTypes());
                continue;
            }

            if (response.action === "request_log_data") {
                appendToolResult("request_log_data", buildMessageDataSummary(response.message_type, response.instance));
                continue;
            }

            if (response.action === "final") {
                finalPayload = response;
                break;
            }

            finalPayload = buildFallbackFinalResponse(rawResponse, response);
            break;
        }

        if (!finalPayload) {
            finalPayload = buildFallbackFinalResponse(lastRawResponse, lastStructuredResponse);
        }

        showThinkingMessage(false);
        finalizeStreamingAssistantMessage(finalPayload.markdown || "No answer returned.");
        updateVisualization({
            summary: finalPayload.summary || "Analysis complete.",
            issues: Array.isArray(finalPayload.issues) ? finalPayload.issues : []
        });

        if (finalPayload.plot_request) {
            drawPlotFromRequest(finalPayload.plot_request);
        }
    } catch (error) {
        console.error("Error processing message:", error);
        clearStreamingAssistantMessage();
        addChatMessage(`Sorry, there was an error processing your request: ${error.message}`, "error");
    } finally {
        showThinkingMessage(false);
        setProcessingState(false);
    }
}

function handleSendMessage() {
    const message = messageInput.value.trim();
    if (!message || isProcessing) {
        return;
    }

    addChatMessage(message, "user");
    messageInput.value = "";
    processUserMessage(message);
}

document.addEventListener("DOMContentLoaded", () => {
    chatMessages = document.getElementById("chatMessages");
    fileInput = document.getElementById("fileInput");
    messageInput = document.getElementById("messageInput");
    sendButton = document.getElementById("sendBtn");
    fileUploadLabel = document.querySelector(".file-upload-label");
    vizArea = document.getElementById("vizArea");
    const settingsBtn = document.getElementById("updateAssistantBtn");

    if (settingsBtn) {
        settingsBtn.textContent = "AI Settings";
        settingsBtn.addEventListener("click", showSettingsPrompt);
    }

    fileInput.addEventListener("change", handleFileUpload);
    sendButton.addEventListener("click", handleSendMessage);
    messageInput.addEventListener("keydown", (event) => {
        if (event.key === "Enter" && !event.shiftKey) {
            event.preventDefault();
            handleSendMessage();
        }
    });

    initializeApp();
});
