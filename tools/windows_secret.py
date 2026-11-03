"""Windows-user DPAPI for the WSL GUI; plaintext travels only over stdin/stdout."""
import base64
import json
from pathlib import Path
import subprocess

POWERSHELL = Path('/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe')


def crypt(value, decrypt=False):
    if not POWERSHELL.exists():
        raise OSError('Windows 암호화 저장은 Windows 또는 WSL에서 사용할 수 있습니다.')
    script = '''
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Security
$request = [Console]::In.ReadToEnd() | ConvertFrom-Json
$bytes = [Convert]::FromBase64String($request.value)
if ($request.decrypt) {
  $result = [Security.Cryptography.ProtectedData]::Unprotect($bytes, $null, [Security.Cryptography.DataProtectionScope]::CurrentUser)
} else {
  $result = [Security.Cryptography.ProtectedData]::Protect($bytes, $null, [Security.Cryptography.DataProtectionScope]::CurrentUser)
}
[Console]::Out.Write([Convert]::ToBase64String($result))
'''
    payload = value if decrypt else base64.b64encode(value.encode('utf-8')).decode('ascii')
    completed = subprocess.run(
        [str(POWERSHELL), '-NoProfile', '-NonInteractive', '-EncodedCommand',
         base64.b64encode(script.encode('utf-16-le')).decode('ascii')],
        input=json.dumps({'value': payload, 'decrypt': decrypt}),
        capture_output=True, text=True, encoding='utf-8', errors='replace', timeout=15)
    if completed.returncode:
        raise OSError('Windows 사용자 암호화 저장소 처리에 실패했습니다.')
    result = completed.stdout.strip()
    return base64.b64decode(result).decode('utf-8') if decrypt else result
