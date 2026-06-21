@echo off
start /b python -m http.server 8080 --directory "f:\JeCH_AeroTECH\JECH_UI\Precision-land\webtools" >nul 2>&1
timeout /t 1 >nul
start http://localhost:8080/PIDReview/index.html
