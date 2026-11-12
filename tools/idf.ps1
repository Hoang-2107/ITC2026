# Chạy idf.py với môi trường ESP-IDF, tự tìm cổng COM của ESP32 khi cần.
# Ví dụ: .\tools\idf.ps1 build   |   .\tools\idf.ps1 flash   |   .\tools\idf.ps1 flash monitor
param([Parameter(ValueFromRemainingArguments = $true)][string[]]$IdfArgs)

. "C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1" *> $null
Set-Location (Split-Path $PSScriptRoot)

$needPort = $IdfArgs | Where-Object { $_ -in 'flash', 'app-flash', 'monitor', 'erase-flash' }
if ($needPort -and -not ($IdfArgs -contains '-p')) {
    $devs = Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -match '\(COM\d+\)' }
    # Ưu tiên: USB-JTAG của ESP32-S3 > USB khác của Espressif > chip nạp CH340/CP210x/FTDI
    $dev = ($devs | Where-Object { $_.DeviceID -match 'VID_303A&PID_1001' }) |
        Select-Object -First 1
    if (-not $dev) { $dev = $devs | Where-Object { $_.DeviceID -match 'VID_303A' } | Select-Object -First 1 }
    if (-not $dev) { $dev = $devs | Where-Object { $_.DeviceID -match 'VID_1A86|VID_10C4|VID_0403' } | Select-Object -First 1 }

    if (-not $dev) {
        Write-Host "LỖI: Không tìm thấy ESP32 nào. Kiểm tra cáp USB (phải là cáp dữ liệu) và cổng cắm." -ForegroundColor Red
        exit 1
    }
    $null = $dev.Name -match '(COM\d+)'
    $port = $Matches[1]
    Write-Host "Dùng cổng $port  ($($dev.Name))" -ForegroundColor Cyan
    $IdfArgs = @('-p', $port) + $IdfArgs
}

# Monitor cũ (mở ở terminal khác) đang giữ cổng COM -> nạp sẽ báo "port is busy". Đóng nó trước.
if ($needPort) {
    $portArg = ($IdfArgs | Select-Object -Index (([array]::IndexOf($IdfArgs, '-p')) + 1))
    $old = Get-CimInstance Win32_Process -Filter "Name='python.exe'" |
        Where-Object { $_.CommandLine -match 'monitor' -and $_.CommandLine -match "-p\s+$portArg(\s|$)" }
    if ($old) {
        Write-Host "Đóng Monitor cũ đang giữ cổng $portArg ..." -ForegroundColor Yellow
        $old | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
        # Chờ Windows nhả cổng COM (tối đa 5 giây) rồi mới nạp
        for ($i = 0; $i -lt 25; $i++) {
            Start-Sleep -Milliseconds 200
            try {
                $sp = New-Object System.IO.Ports.SerialPort $portArg
                $sp.Open(); $sp.Close(); $sp.Dispose()
                break
            } catch { }
        }
    }
}

Write-Host "> idf.py $($IdfArgs -join ' ')" -ForegroundColor Cyan
idf.py @IdfArgs
$code = $LASTEXITCODE
if ($code -eq 0) {
    Write-Host "`n=== THÀNH CÔNG ===" -ForegroundColor Green
} else {
    Write-Host "`n=== THẤT BẠI (mã lỗi $code) – xem thông báo phía trên ===" -ForegroundColor Red
    if ($needPort) {
        Write-Host "Nếu báo 'port is busy': đóng mọi cửa sổ Monitor / Serial Monitor khác đang mở cổng COM, rồi nạp lại." -ForegroundColor Yellow
        Write-Host "Nếu báo 'Failed to connect': giữ BOOT, nhấn rồi thả RESET, thả BOOT, rồi nạp lại." -ForegroundColor Yellow
    }
}
exit $code
