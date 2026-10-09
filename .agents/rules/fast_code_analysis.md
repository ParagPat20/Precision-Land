# Fast Code Analysis & Structural Search Rule

To ensure ultra-fast file analysis, prevent token-streaming latency, and avoid shell escaping conflicts:

1. **Always Use `cmd.exe` over PowerShell for `ast-grep`:**
   - **Reason:** PowerShell interprets `$` (e.g., `$NAME:`, `$ARGS`, `$$$BODY`) as PowerShell variable references and throws syntax errors (`Variable reference is not valid. ':' was not followed by a valid variable name`).
   - `cmd.exe` does not use `$` for variable expansion (it uses `%VAR%`), allowing AST pattern meta-variables to pass through directly without interference.
   - When invoking via terminal or runner:
     ```cmd
     cmd.exe /c 'set PATH=C:\Users\jecha\.local\bin;%PATH% && ast-grep run --pattern "def $NAME($$$ARGS): $$$" <path>'
     ```
   - Note the single quotes `'...'` enclosing the command string so the outer PowerShell wrapper does not interpolate `$` symbols.

2. **AST Pattern Syntax Guidelines:**
   - **Functions:** `def $NAME($$$ARGS): $$$` (The trailing `$$$` represents the statement block body).
   - **Classes:** `class $NAME: $$$` or `class $NAME($$$BASE): $$$`.
   - **Imports:** `import $PKG` or `from $PKG import $$$`.
   - **Calls:** `$OBJ.$METHOD($$$ARGS)`.

3. **Selective File Inspection:**
   - Pinpoint line numbers using `ast-grep` first, then inspect only the targeted line slices using file viewing tools rather than reading whole files sequentially.

4. **Execution Binaries:**
   - Native binary: `C:\Users\jecha\.local\bin\ast-grep.exe`
   - Batch wrapper: `C:\Users\jecha\.local\bin\ast.cmd`
   - Global MCP Server: Registered in `C:\Users\jecha\.gemini\config\mcp_config.json`.

5. **Single-Shot Directory Analysis (No Iterative File-by-File Loops):**
   - When asked to analyze, explore, or inspect an entire directory:
     **DO NOT** traverse subfolders one-by-one with sequential `list_dir` and `view_file` calls. That produces 15+ slow network round-trips.
   - **Instead, run a single Repomix command:**
     - For full compressed AST code structure:
       ```cmd
       cmd.exe /c "npx.cmd -y repomix <path> --compress --no-security-check --output-show-line-numbers --stdout"
       ```
     - For quick directory tree and metadata:
       ```cmd
       cmd.exe /c "npx.cmd -y repomix <path> --no-files --stdout"
       ```
   - This parses the entire folder locally on the CPU with Tree-sitter in ~1–2 seconds and returns the whole structural summary in a single tool call.
