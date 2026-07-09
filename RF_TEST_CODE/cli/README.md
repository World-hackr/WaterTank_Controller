# Arduino CLI Helpers

These scripts wrap Arduino CLI for the ATtiny402 board used in this project.

List connected ports:

```powershell
.\RF_TEST_CODE\cli\list_ports.ps1
```

Compile a sketch at the default 10 MHz clock:

```powershell
.\RF_TEST_CODE\cli\compile.ps1 RF_TEST_CODE\11_auth_standalone_tx
```

Compile at another clock:

```powershell
.\RF_TEST_CODE\cli\compile.ps1 RF_TEST_CODE\11_auth_standalone_tx -Clock 5internal
```

Upload with SerialUPDI:

```powershell
.\RF_TEST_CODE\cli\upload.ps1 RF_TEST_CODE\11_auth_standalone_tx -Port COM13
```

Upload at 5 MHz:

```powershell
.\RF_TEST_CODE\cli\upload.ps1 RF_TEST_CODE\11_auth_standalone_tx -Port COM13 -Clock 5internal
```

If your UPDI adapter is slower or unreliable, try:

```powershell
.\RF_TEST_CODE\cli\upload.ps1 RF_TEST_CODE\11_auth_standalone_tx -Port COM13 -Programmer serialupdi57k
```

Current default:

- Board: `megaTinyCore:megaavr:atxy2`
- Chip: `ATtiny402`
- Clock: `10 MHz internal`
- millis/micros: enabled
- Programmer: `serialupdi`
