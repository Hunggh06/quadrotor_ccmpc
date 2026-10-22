# Quadrotor CC-MPC

Mô phỏng quadrotor tránh vật cản bằng **Chance-Constrained Model Predictive
Control (CC-MPC)** trên MuJoCo, với mô hình Crazyflie 2.

Code hiện tại được đồng bộ từ `ccmpc_project/` vào **[`2.Code/`](2.Code/)**.
Cấu trúc package hiện dùng là `simulation` và `visualization`, thay cho package
`quadrotor_mpc` của phiên bản trước.

## Thành phần chính

- **NMPC 13-state quaternion** dùng do-mpc/CasADi, kết nối plant MuJoCo.
- Cảm biến mô phỏng có seed, ESEKF 12D, tracker vật cản và lan truyền covariance.
- Các chế độ `deterministic`, `individual`, `joint_uniform` trong CLI native.
- **Joint-risk NLP thử nghiệm**: tối ưu phân bổ rủi ro theo horizon; chạy riêng
  qua `experiments/joint_risk/`, không thay thế baseline mặc định.
- Viewer MuJoCo, panel Qt, chạy headless, lưu telemetry, xuất video và đồ thị.
- Bộ điều khiển CC-MPC 9-state/CVXPY được giữ làm tham chiếu riêng.

> Đây là code nghiên cứu/mô phỏng, không phải bộ điều khiển bay thực tế.
> Ngân sách rủi ro theo horizon không tự tạo bảo đảm an toàn cho toàn bộ chuyến
> bay. Các kết quả cũ trong repo không đại diện cho bản code hiện tại.

## Cài đặt và chạy nhanh

Cần Python **3.10+** và [uv](https://docs.astral.sh/uv/).

```bash
git clone https://github.com/Hunggh06/quadrotor_ccmpc.git
cd quadrotor_ccmpc/2.Code
uv sync --extra full --extra dev --locked
```

Kiểm tra cấu hình mà không chạy mô phỏng:

```bash
uv run quadrotor-mpc-native --config config/mujoco_native_ccmpc.yaml --validate-config
```

Chạy CC-MPC không cửa sổ:

```bash
uv run quadrotor-mpc-native --config config/mujoco_native_ccmpc.yaml --headless --mode joint_uniform --seed 1001 --duration 10
```

Mở viewer MuJoCo và panel Qt:

```bash
uv run quadrotor-mpc-native --config config/mujoco_native_ccmpc.yaml
```

Viewer cần môi trường đồ họa/OpenGL phù hợp. Kết quả chạy được tạo trong
`2.Code/outputs/` và không được đưa vào Git.

## Cấu trúc repository

```text
2.Code/                         Code hiện tại
├── simulation/                 Điều khiển, ước lượng, plant, runtime, CLI/Qt
├── visualization/              Đồ thị và báo cáo từ bản ghi
├── config/                     Cấu hình YAML và kịch bản thí nghiệm
├── models/                     Crazyflie 2 XML, mesh và giấy phép model
├── experiments/                Joint-risk, ablation và phân tích nhiều seed
├── scripts/                    Xuất video, báo cáo và hình minh họa
├── tests/                      Kiểm thử của bản hiện tại
├── pyproject.toml
└── uv.lock
1.Docs/                         Tài liệu cũ, giữ nguyên
3.Notebooks/                    Notebook cũ, giữ nguyên
assets/                         Tài nguyên cũ, giữ nguyên
```

Xem **[hướng dẫn code](2.Code/README.md)** để chạy joint-risk, xuất kết quả và
kiểm thử. Tài liệu, notebook và launcher `Quadrotor MPC Workbench.bat` của bản
cũ chưa được chuyển sang package mới; không dùng chúng làm hướng dẫn chạy bản này.

## Phạm vi bản cập nhật

Chỉ đồng bộ mã nguồn, cấu hình, model cần chạy, dependency lock và hướng dẫn;
không upload môi trường ảo, cache, dữ liệu/kết quả thí nghiệm, PDF hay slide.
CI được điều chỉnh để kiểm tra đúng package và bộ test hiện tại.

## Giấy phép

Code giữ [giấy phép MIT](LICENSE) và thông tin tác giả gốc. Model Crazyflie 2 có
[giấy phép riêng](2.Code/models/bitcraze_crazyflie_2/LICENSE).
