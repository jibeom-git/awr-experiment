# experiment/visualize.py
# AGV 경사면 판단 및 적응형 속도 제어 정보 추출을 위한 시각화 엔진

import os
import sys
import pandas as pd
import numpy as np
from typing import Optional

# Pylance 모듈 소스 누락 경고 방지 및 폴백 가이드 수립
try:
    import matplotlib.pyplot as plt
    import seaborn as sns
    _VIS_AVAILABLE = True
except ImportError:
    _VIS_AVAILABLE = False
    print("[Warning] 시각화 의존성 모듈(matplotlib, seaborn)이 누락되었습니다. pip install을 진행하십시오.")

class AGVDrivingVisualizer:
    """
    AGV 주행 데이터 로그를 역추적하여, 경사면 극복 과정에서의 
    피치 변위량 및 물리 센서 특성 변화를 학술 보고서 수준으로 시각화하는 클래스.
    """
    def __init__(self, csv_path: str):
        self.csv_path: str = csv_path
        # Pylance의 'None' 유형 아래 첨자 사용 불가(Optional 지적) 문제를 해결하기 위해 타겟 타입 명시
        self.df: Optional[pd.DataFrame] = None
        self._load_and_preprocess()

    def _load_and_preprocess(self) -> None:
        """내부 데이터 로드 및 시계열 연속성 확보를 위한 전처리 전담 로직"""
        if not os.path.exists(self.csv_path):
            raise FileNotFoundError(f"[Error] 데이터 파일이 존재하지 않습니다: {self.csv_path}")
        
        loaded_df = pd.read_csv(self.csv_path)
        # 문자열 타임스탬프를 연산 가능한 datetime 구조로 파싱
        loaded_df['timestamp'] = pd.to_datetime(loaded_df['timestamp'])
        # 시계열 추이 분석의 가시성을 위해 정렬 수행
        self.df = loaded_df.sort_values('timestamp').reset_index(drop=True)

    def plot_slope_control_analysis(self, save_path: str = "slope_control_analysis.png") -> None:
        """
        [시각화 1] 경사면 진입 시 Pitch 가속도 변위와 Speed Command 간의 메커니즘 분석
        - 상단: Pitch 및 Pitch_Delta 변위 추이를 통한 경사면 물리 진입 판단 근거 시각화
        - 하단: 주행 결과(Result) 분포에 따른 실시간 제어 속도(speed_cmd) 매핑 수치 증명
        """
        if not _VIS_AVAILABLE:
            print("[Error] 라이브러리 부재로 plot_slope_control_analysis 기능을 실행할 수 없습니다.")
            return
            
        # 정적 분석 엔지 Pylance에게 self.df가 확실히 DataFrame임을 증명 (None 검증 단언문 수립)
        assert self.df is not None, "데이터프레임이 초기화되지 않았습니다."
        
        # 스타일 및 서체 엄격 격식 수립 (이모지 및 불안정 스타일 배제)
        plt.rc('font', family='NanumGothic')
        plt.rcParams['axes.unicode_minus'] = False
        sns.set_theme(style="darkgrid")

        fig, axes = plt.subplots(2, 1, figsize=(14, 10), sharex=False)

        # ── 1. 경사면 특성 그룹화 및 피치-가속도 데이터 추출 ──
        slope_data = self.df[self.df['label'].isin(['정상 평지', '10도 언덕', '20도 언덕'])].copy()
        
        # 차트 가독성을 위해 5개씩 다운샘플링하여 라인 중첩 방지
        slope_data = slope_data.iloc[::5] 

        # 첫 번째 서브플롯: Pitch 각도와 Y축 진행 방향 가속도 변위 추이
        ax1_twin = axes[0].twinx()
        sns.lineplot(data=slope_data, x=slope_data.index, y='pitch', ax=axes[0], color='#1f77b4', label='Pitch (기울기 각도)', linewidth=2)
        
        # 💡 [Pylance 해결] 문자열 앞에 r을 붙여 LaTeX 수식 백슬래시 이스케이프 오류 제거
        sns.lineplot(data=slope_data, x=slope_data.index, y='pitch_delta', ax=ax1_twin, color='#ff7f0e', label='Pitch Delta (기울기 변화율)', linewidth=1.5, alpha=0.7)
        
        axes[0].set_title(r"경사도 진입에 따른 차체 Pitch 물리 변위 및 실시간 변화율 ($Pitch\_Delta$) 추출", fontsize=13, fontweight='bold')
        axes[0].set_ylabel(r"Pitch Angle ($^{\circ}$)", fontsize=11, fontweight='bold')
        ax1_twin.set_ylabel(r"Pitch Delta ($^{\circ}/s$)", fontsize=11, fontweight='bold')
        
        # 레이블 범례 통합 표시 프로세스
        lines1, labels1 = axes[0].get_legend_handles_labels()
        lines2, labels2 = ax1_twin.get_legend_handles_labels()
        axes[0].legend(lines1 + lines2, labels1 + labels2, loc='upper left')

        # 두 번째 서브플롯: 주행 상태 결과별 제어 속도(speed_cmd)와 수직 진동 충격 분산 관계 파악
        sns.violinplot(data=self.df, x='result', y='speed_cmd', ax=axes[1], palette='muted', inner='quartile', linewidth=1.5)
        sns.stripplot(data=self.df, x='result', y='speed_cmd', ax=axes[1], color='black', size=2, alpha=0.3, jitter=0.2)
        
        axes[1].set_title(r"최종 주행 결과 ($Result$) 상태 앙상블에 따른 제어 인가 속도 ($speed\_cmd$) 분산 분석", fontsize=13, fontweight='bold')
        axes[1].set_xlabel("AGV 최종 주행 상태 레이블 (Result)", fontsize=11, fontweight='bold')
        axes[1].set_ylabel("Applied Speed PWM Command", fontsize=11, fontweight='bold')

        plt.tight_layout()
        plt.savefig(save_path, dpi=300)
        plt.close()
        print(f"[시각화 완료] 경사면 극복 메커니즘 분석 차트가 '{save_path}' 경로에 저장되었습니다.")

    def plot_anomaly_feature_space(self, save_path: str = "anomaly_feature_space.png") -> None:
        """
        [시각화 2] Isolation Forest 및 XGBoost가 학습할 다차원 피처 공간의 관계성 추출
        - 경사면 등판 실패(hill_fail) 및 슬립(slip)을 결정짓는 핵심 물리 파라미터 간의 상호 상관지도 도출.
        """
        if not _VIS_AVAILABLE:
            print("[Error] 라이브러리 부재로 plot_anomaly_feature_space 기능을 실행할 수 없습니다.")
            return

        assert self.df is not None, "데이터프레임이 초기화되지 않았습니다."
        plt.rc('font', family='NanumGothic')
        sns.set_theme(style="white")

        # 분석용 핵심 타깃 피처 선별
        feature_cols = ['pitch', 'weight', 'accel_y', 'accel_z', 'pitch_delta']
        
        # 💡 [Pylance 해결] 명확한 df 참조를 보장하여 'copy'은 'None'의 알려진 특성이 아님 에러 차단
        plot_df = self.df.copy()
        
        # 데이터 포인트 중첩 방지 및 샘플 밀도 최적화를 위한 1000개 다운샘플링 수립
        if len(plot_df) > 1000:
            plot_df = plot_df.sample(n=1000, random_state=42)

        g = sns.pairplot(
            plot_df, 
            vars=feature_cols, 
            hue='result', 
            palette='Set1',
            diag_kind='kde',
            plot_kws={'alpha': 0.6, 's': 25, 'edgecolor': 'k', 'linewidth': 0.5}
        )
        g.fig.suptitle(r"AGV 제어 인지 차원 확장을 위한 핵심 물리 센서 간 가용 피처 공간(Feature Space) 산점도 행렬", y=1.02, fontsize=14, fontweight='bold')
        
        plt.savefig(save_path, dpi=300)
        plt.close()
        print(f"[시각화 완료] 머신러닝 피처 데이터 공간 분포도가 '{save_path}' 경로에 저장되었습니다.")

# visualize.py 파일의 맨 아래 코드를 이 구조로 교체하십시오.

if __name__ == "__main__":
    # 1. 현재 실행 중인 visualize.py 파일의 절대 디렉토리 파악 (~/insite/experiment)
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    
    # 2. find 명령어로 도출된 실제 하위 데이터 경로를 절대 경로 체계로 빌드
    # SCRIPT_DIR이 ~/insite/experiment 이므로 아래 연산 시 ~/insite/experiment/data/raw_experiment.csv 가 됩니다.
    REAL_DATA_PATH = os.path.join(SCRIPT_DIR, "data", "raw_experiment.csv")
    
    # 3. 경로 검증 및 최종 바인딩
    if os.path.exists(REAL_DATA_PATH):
        target_csv = REAL_DATA_PATH
        print(f"[Engine] 실측 실험 데이터를 감지하여 로드합니다: {target_csv}")
    else:
        # 혹시 모를 폴백을 대비한 다중 경로 앙상블 탐색
        candidate_paths = [
            os.path.join(SCRIPT_DIR, "raw_experiment.csv"),
            "raw_experiment.csv",
            "../raw_experiment.csv"
        ]
        target_csv = "raw_experiment.csv"
        for path in candidate_paths:
            if os.path.exists(path):
                target_csv = path
                print(f"[Info] 폴백 경로에서 데이터를 감지했습니다: {target_csv}")
                break
                
    # 4. 시각화 엔진 트리거
    visualizer = AGVDrivingVisualizer(csv_path=target_csv)
    visualizer.plot_slope_control_analysis()
    visualizer.plot_anomaly_feature_space()