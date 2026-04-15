// Helper to add a loading overlay to a page and show/hide it

function init_loading_overlay() {

    let div = document.createElement("div")
    div.id = "loading"
    div.style.position = "fixed"
    div.style.top = 0
    div.style.left = 0
    div.style.width = "100%"
    div.style.height = "100%"
    div.style.backgroundColor = "rgba(255, 255, 255, 0.45)"
    div.style.backdropFilter = "blur(1px)"
    div.style.zIndex = 99
    div.style.visibility = "hidden"

    let panel = document.createElement("div")
    panel.style.position = "absolute"
    panel.style.top = "50%"
    panel.style.left = "50%"
    panel.style.transform = "translate(-50%, -50%)"
    panel.style.display = "flex"
    panel.style.alignItems = "center"
    panel.style.gap = "12px"
    panel.style.padding = "14px 18px"
    panel.style.backgroundColor = "#ffffff"
    panel.style.border = "1px solid #d8dde6"
    panel.style.borderRadius = "6px"
    panel.style.boxShadow = "0 8px 24px rgba(0, 0, 0, 0.12)"
    panel.style.fontFamily = "sans-serif"
    panel.style.fontSize = "15px"
    panel.style.color = "#1f2933"

    let spinner = document.createElement("span")
    spinner.style.width = "18px"
    spinner.style.height = "18px"
    spinner.style.border = "3px solid #d8dde6"
    spinner.style.borderTopColor = "#2f80ed"
    spinner.style.borderRadius = "50%"
    spinner.style.animation = "loading-overlay-spin 0.8s linear infinite"

    let text = document.createElement("span")
    text.innerHTML = "Loading..."

    let style = document.createElement("style")
    style.textContent = "@keyframes loading-overlay-spin { to { transform: rotate(360deg); } }"

    panel.appendChild(spinner)
    panel.appendChild(text)
    div.appendChild(style)
    div.appendChild(panel)
    document.body.appendChild(div)

}

async function loading_call(fun) {

    // Show loading screen
    let overlay = document.getElementById("loading")
    overlay.style.visibility = 'visible'

    // https://forum.freecodecamp.org/t/how-to-make-js-wait-until-dom-is-updated/122067/2

    // this double requestAnimationFrame ensures that the loading screen is rendered before the load starts
    async function run() {
        // Call the passed in function
        await fun()

        // Hide loading screen
        overlay.style.visibility = 'hidden'
    }

    async function intermediate() {
        window.requestAnimationFrame(run)
    }

    window.requestAnimationFrame(intermediate)

}
