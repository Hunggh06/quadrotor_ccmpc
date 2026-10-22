# CC-MPC simulation code

Thư mục này chứa code hiện tại của `ccmpc_project`. Chạy các lệnh dưới đây từ
`2.Code/`, không phải từ thư mục gốc repository.

## Cài đặt

Python 3.10+; Python 3.12 được dùng để kiểm tra bản đồng bộ.

```bash
uv sync --extra full --extra dev --locked
```

Hoặc cài bằng pip trong môi trường ảo đã kích hoạt:

```bash
python -m pip install -e ".[full,dev]"
```

Extra `ui` cung cấp do-mpc, CasADi và Qt; `mujoco` cung cấp plant/renderer;
`video` cung cấp các gói xuất video. `full` gộp các nhóm chạy này, còn `dev`
cung cấp công cụ kiểm thử/build. Không cần cài extra `notebooks` để chạy code.

## Cấu trúc

| Đường dẫn | Vai trò |
|---|---|
| `simulation/core/` | Belief/control contracts, tham số xe, va chạm, timing |
| `simulation/control/ccmpc_13state/` | NMPC quaternion, chance constraints, covariance, safety fallback |
| `simulation/control/ccmpc_13state/experimental_joint_risk/` | Joint-risk NLP thử nghiệm, chỉ bật khi được yêu cầu |
| `simulation/control/ccmpc_9state/` | Track CVXPY 9-state tham chiếu |
| `simulation/estimation/` | ESEKF và tracker vật cản; truth baseline |
| `simulation/infrastructure/` | Resource lookup và adapter MuJoCo |
| `simulation/application/` | Runtime native, telemetry, replay, validation helpers |
| `simulation/interfaces/` | CLI native, viewer MuJoCo và panel Qt |
| `visualization/reporting/` | Đồ thị từ bản ghi |
| `config/` | Cấu hình controller/native và kịch bản YAML |
| `models/bitcraze_crazyflie_2/` | XML, mesh và giấy phép Crazyflie 2 |
| `experiments/` | Runner, ablation, joint-risk, phân tích và export |
| `scripts/` | Xuất video, báo cáo và hình minh họa |
| `tests/` | Kiểm thử joint-risk, export và phân tích paired-seed |

Giữ nguyên tên package `simulation` và `visualization`: code, entry point và
resource lookup đều dùng cấu trúc này. Một số thư mục là namespace package
(PEP 420) nên không có `__init__.py`.

## Mô phỏng native

```bash
# Kiểm tra cấu hình, không chạy solver hay viewer
uv run quadrotor-mpc-native --config config/mujoco_native_ccmpc.yaml --validate-config

# Headless, có ghi telemetry
uv run quadrotor-mpc-native --config config/mujoco_native_ccmpc.yaml --headless --mode joint_uniform --seed 1001 --duration 10

# Viewer MuJoCo + panel Qt
uv run quadrotor-mpc-native --config config/mujoco_native_ccmpc.yaml

# Chỉ mở viewer, không mở panel
uv run quadrotor-mpc-native --config config/mujoco_native_ccmpc.yaml --no-panel
```

`--mode` nhận `deterministic`, `individual`, `joint_uniform`. Các tùy chọn khác:
`--noise-scale`, `--duration`, `--seed`, `--camera follow`, `--camera fixed`,
`--no-record`, `--progress-json` (headless). Xem đầy đủ bằng `--help`.

Mỗi lần chạy có thể tạo `telemetry.csv`, `trajectory.npz`, `scenario.yaml`,
`summary.yaml` và event log trong `outputs/native/<run>/`.

## Joint-risk thử nghiệm

`joint_uniform` trong CLI native là **baseline chia đều rủi ro**, không phải
joint-risk NLP tối ưu phân bổ. Để so sánh baseline với joint-risk NLP:

```bash
uv run python experiments/joint_risk/run_joint_risk.py --scenario five_lowu_narrow --variant uniform --variant hard_uniform --variant joint_hard --seed 1001 --duration 6 --tag demo
```

- `uniform`: CC-MPC chia đều rủi ro, giữ thiết lập soft/hard của YAML.
- `hard_uniform`: chia đều rủi ro, hard chance constraints.
- `joint_soft`: joint-risk NLP với relaxation mềm.
- `joint_hard`: joint-risk NLP với hard chance constraints, formulation `required`.
- `joint_required`: alias của `joint_hard`.

Các scene hiện có: `five_lowu_narrow`, `five_lowu`, `five_through`, `cc_ablation`.
Kết quả và metadata được ghi vào `outputs/joint_risk/`.

Xem kế hoạch benchmark nhiều seed mà chưa chạy solver:

```bash
uv run python experiments/joint_risk/run_joint_multiseed.py --dry-run
```

Bỏ `--dry-run` để chạy kế hoạch 90 trường hợp có hỗ trợ tiếp tục; đây là tác vụ
tốn thời gian. Script kiểm tra fingerprint nguồn/cấu hình trước khi nối kết quả
vào một đợt chạy cũ. Các script phân tích/export cần bản ghi đã tạo ở máy bạn;
repo không chứa sẵn kết quả.

## Replay, video và báo cáo

Thay `<run>` bằng thư mục bản ghi thực tế:

```bash
uv run quadrotor-mpc-native --replay outputs/native/<run>
uv run python scripts/export_native_video.py outputs/native/<run> --camera orbit
uv run python scripts/report_native_run.py outputs/native/<run>
```

Không truyền đường dẫn cho hai script export/report sẽ chọn bản ghi native
mới nhất. Export video phát lại trạng thái đã lưu qua renderer, không giải lại
NMPC. Báo cáo gồm `report.png`, `report_3d.html`, `report_timeseries.html`.

## Kiểm thử và build

```bash
uv run python -m unittest discover -s tests -v
uv run python -m build
```

Test gồm kiểm tra phép nội suy/hiển thị, thống kê paired-seed bằng dữ liệu tổng
hợp, và các bài toán joint-risk nhỏ có giải NLP thật. Chạy test cần các gói
solver trong extra `full`, nhưng không cần cửa sổ đồ họa hay dữ liệu output cũ.

## Lưu ý

- Đây là nền tảng nghiên cứu, chưa phải flight controller triển khai thực tế.
- Risk budget được áp dụng theo horizon; không suy ra bảo đảm xác suất va chạm
  cho toàn episode chỉ từ một lần solve hoặc từ kết quả vài seed.
- Soft constraints, solver failure và fallback cần được đọc cùng telemetry;
  không xem trạng thái solver thành công là bằng chứng an toàn tuyệt đối.
- Không đưa `.venv/`, `__pycache__/`, `outputs/`, dữ liệu thử nghiệm hay slide vào Git.
- Các command của workbench/Streamlit/Monte Carlo CLI cũ không thuộc bản code
  rút gọn này. Native CLI duy nhất được đăng ký là `quadrotor-mpc-native`.
- Giữ [MIT license](LICENSE) và [giấy phép model](models/bitcraze_crazyflie_2/LICENSE).
