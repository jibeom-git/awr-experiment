# tests/test_astra_color.py
# Astra RGB 스트림 단독 테스트
# 실행: python tests/astra_rgb.py

import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sensors.astra import AstraCamera

if __name__ == "__main__":
    print("Astra Color 스트림 테스트")
    cam = AstraCamera(enable_color=True, enable_depth=True)

    try:
        print("10프레임 읽기...")
        for i in range(10):
            depth, color = cam.get_synced_frames()

            depth_info = f"shape={depth.shape} center={depth[240,320]}mm" \
                         if depth is not None else "없음"
            color_info = f"shape={color.shape} dtype={color.dtype}" \
                         if color is not None else "없음"

            print(f"[{i+1:2d}] Depth: {depth_info}")
            print(f"      Color: {color_info}")
            time.sleep(0.2)

        print("\nAstra RGB + Depth 정상 동작 확인")

    except KeyboardInterrupt:
        print("\n중단")
    finally:
        cam.close()