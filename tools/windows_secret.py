"""Windows-user DPAPI for the WSL GUI; plaintext travels only over stdin/stdout."""
# WSL에서 실행되는 GUI가 비밀번호를 Windows 사용자 계정의 DPAPI로 암호화/복호화하는 모듈.
# PowerShell을 호출하며, 평문은 명령줄이 아닌 stdin/stdout으로만 주고받는다.
import base64
import json
from pathlib import Path
import subprocess

POWERSHELL = Path('/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe')


def crypt(value, decrypt=False):
    """value를 암호화(기본)하거나 decrypt=True면 복호화한다.
    암호화 결과는 base64 문자열이며, 같은 Windows 사용자만 다시 풀 수 있다.
    """
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
    # 암호화할 평문은 base64로, 복호화할 값은 이미 base64이므로 그대로 보낸다.
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
