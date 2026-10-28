@echo off
cd /d "%~dp0"
py serve.py || python serve.py
