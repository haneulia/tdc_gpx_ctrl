# Vivado XSIM Artifact Policy

이 폴더는 Vivado/xsim 실행 산출물을 루트 디렉터리에 쌓지 않기 위한 보관 영역이다.

## 표준 구조

```text
sim_results/
  vivado_xsim/
    README.md
    sessions/
      <YYMMDDHHMMSS>_<label>/
        logs/
          compile/     xvlog, xvhdl log
          elaborate/   xelab log
          simulate/    xsim log
          vivado/      Vivado batch/project log
          journal/     *.jou
        waves/         *.wdb
        work/          *.pb, xsim.dir, .Xil, legacy sim work dirs
        tmp/           script-generated temporary files
        crash/         hs_err_pid*.log/dmp
        manifest.csv
        README.md
```

## 운영 규칙

- Vivado 기준 경로는 `C:\AMDDesignTools\2025.2.1\Vivado`로 유지한다.
- 시뮬레이션 실행 스크립트는 완료 또는 실패 후 `scripts/archive_vivado_xsim_outputs.ps1`를 호출한다.
- 기본 정책은 삭제가 아니라 session 폴더로 이동 보관이다.
- 문서에 xsim 근거를 남길 때는 session 경로와 log line을 함께 기록한다.
- `sessions/` 아래 산출물은 재생성 가능한 실행 산출물이므로 Git에 포함하지 않는다.
