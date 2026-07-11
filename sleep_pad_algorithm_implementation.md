# Sleep Pad 呼吸心率算法实现指南

## 🏗️ 系统架构

```
原始数据 (90Hz) 
    ↓
[预处理模块] → 带通滤波(0.5-4Hz) + 降采样
    ↓
[多通道处理] → 4通道并行处理 + SQI计算
    ↓
[心率检测引擎] → 频域+时域+自相关 三重估计
    ↓
[呼吸检测引擎] → 低频提取 + J波幅度调制
    ↓
[融合仲裁器] → 多方法共识 + 异常处理
    ↓
[后处理] → 卡尔曼滤波 + 连续性验证
    ↓
输出: 心率(bpm) + 呼吸率(rpm)
```

## 📝 实现阶段规划

### 阶段1: 数据预处理管道 (1-2周)

**1.1 数据读取模块**
```cpp
class DataLoader {
public:
    // 读取16位小端序二进制数据
    std::vector<int16_t> loadSensorData(const std::string& filename);
    // 时间戳解析
    uint64_t parseTimestamp(const std::string& filename);
};
```

**1.2 滤波器设计**
```cpp
class BandpassFilter {
private:
    // IIR滤波器系数 (0.5-4.0Hz @ 90Hz采样率)
    double a_coeffs[5], b_coeffs[5];
    double delay_line[5];
    
public:
    double process(double input);
    void reset();
};
```

**1.3 多通道缓冲区**
```cpp
class MultiChannelBuffer {
    static const int CHANNELS = 4;
    static const int BUFFER_SIZE = 1350; // 15秒 @ 90Hz
    
    CircularBuffer<double> channels[CHANNELS];
    std::atomic<uint64_t> timestamp;
};
```

### 阶段2: 信号质量评估 (1周)

**2.1 SQI计算器**
```cpp
class SQICalculator {
public:
    struct SQIResult {
        double hr_band_energy;    // 心率频带能量
        double noise_energy;      // 噪声能量
        double sqi_score;         // 质量分数(0-1)
        bool is_valid;           // 是否有效
    };
    
    SQIResult calculateSQI(const std::vector<double>& signal);
};
```

**核心算法:**
```
SQI = Energy_HR_Band / (Energy_Total - Energy_HR_Band + ε)
```

### 阶段3: 心率检测引擎 (2-3周)

**3.1 频域估计器**
```cpp
class FrequencyEstimator {
private:
    FFTProcessor fft_processor;
    static const int WINDOW_SIZE = 1350; // 15秒窗口
    
public:
    struct HeartRateResult {
        double heart_rate;
        double confidence;
        double peak_frequency;
    };
    
    HeartRateResult estimate(const std::vector<double>& signal);
};
```

**3.2 时域峰检测器**
```cpp
class PeakDetector {
private:
    static const int SLIDING_WINDOW_MS = 6;
    double adaptive_threshold;
    
public:
    struct PeakInfo {
        uint64_t timestamp;
        double amplitude;
        double quality_score;
    };
    
    std::vector<PeakInfo> detectJPeaks(const std::vector<double>& signal);
};
```

**3.3 自相关估计器**
```cpp
class AutoCorrelationEstimator {
public:
    struct ACResult {
        double heart_rate;
        double correlation_peak;
        std::vector<double> correlation_function;
    };
    
    ACResult estimate(const std::vector<double>& signal);
};
```

**3.4 多通道共识机制**
```cpp
class ConsensusEngine {
private:
    static const int MIN_CHANNELS = 3; // N=3投票阈值
    
public:
    struct ConsensusResult {
        bool valid;
        uint64_t timestamp;
        double confidence;
        std::vector<int> supporting_channels;
    };
    
    ConsensusResult validateHeartbeat(
        const std::vector<PeakInfo>& channel_peaks,
        const std::vector<double>& sqi_scores
    );
};
```

### 阶段4: 呼吸检测引擎 (2周)

**4.1 低频呼吸提取(主方法)**
```cpp
class RespirationExtractor {
private:
    LowPassFilter lpf_filter;    // 0.1-0.5Hz
    Decimator decimator;         // 90Hz → 5Hz
    
public:
    struct RespirationResult {
        double respiration_rate;
        double amplitude;
        std::vector<double> respiration_signal;
    };
    
    RespirationResult extractFromRawSignal(const std::vector<double>& signal);
};
```

**4.2 J波幅度调制法(辅助验证)**
```cpp
class AmplitudeModulationExtractor {
public:
    RespirationResult extractFromJPeakAmplitudes(
        const std::vector<PeakInfo>& j_peaks
    );
};
```

### 阶段5: 融合仲裁器 (1-2周)

**5.1 多估计器仲裁**
```cpp
class HeartRateArbitrator {
public:
    struct FinalResult {
        double heart_rate;
        double confidence;
        EstimatorType primary_source; // FREQUENCY/PEAK/AUTOCORR
        bool backup_activated;
    };
    
    FinalResult arbitrate(
        const FrequencyEstimator::HeartRateResult& freq_result,
        const std::vector<PeakInfo>& peak_result,
        const AutoCorrelationEstimator::ACResult& ac_result,
        double average_sqi
    );
};
```

**5.2 呼吸率共识**
```cpp
class RespirationArbitrator {
public:
    double arbitrateRespirationRate(
        const RespirationResult& low_freq_result,
        const RespirationResult& amplitude_mod_result
    );
};
```

### 阶段6: 后处理与连续性验证 (1周)

**6.1 卡尔曼滤波器**
```cpp
class KalmanSmoother {
private:
    double state;           // 当前状态估计
    double covariance;      // 状态协方差
    double process_noise;   // 过程噪声
    double measurement_noise; // 测量噪声
    
public:
    double update(double measurement);
    void reset();
};
```

**6.2 连续性验证器**
```cpp
class ContinuityValidator {
private:
    std::deque<double> history; // 历史心率队列
    static const int HISTORY_SIZE = 5;
    
public:
    struct ValidationResult {
        bool is_valid;
        double corrected_value;
        std::string reason;
    };
    
    ValidationResult validate(double current_hr);
};
```

## 🔧 核心算法实现

### 心率检测核心逻辑
```cpp
class HeartRateEngine {
public:
    struct HeartRateOutput {
        double heart_rate;
        double confidence;
        uint64_t timestamp;
        std::vector<double> rr_intervals; // 用于HRV分析
    };
    
    HeartRateOutput processChannels(
        const std::array<std::vector<double>, 4>& filtered_channels
    ) {
        // 1. 计算各通道SQI
        std::array<double, 4> sqi_scores;
        for(int i = 0; i < 4; i++) {
            sqi_scores[i] = sqi_calc.calculateSQI(filtered_channels[i]).sqi_score;
        }
        
        // 2. 并行运行三种估计器
        auto freq_result = freq_estimator.estimate(getBestChannel(filtered_channels, sqi_scores));
        auto peak_results = detectPeaksAllChannels(filtered_channels);
        auto ac_result = ac_estimator.estimate(getBestChannel(filtered_channels, sqi_scores));
        
        // 3. 多通道共识验证
        auto consensus = consensus_engine.validateHeartbeat(peak_results, sqi_scores);
        
        // 4. 仲裁最终结果
        return arbitrator.arbitrate(freq_result, peak_results, ac_result, 
                                  getAverageSQI(sqi_scores));
    }
};
```

### 呼吸检测核心逻辑
```cpp
class RespirationEngine {
public:
    struct RespirationOutput {
        double respiration_rate;
        double amplitude;
        double confidence;
    };
    
    RespirationOutput processSignal(
        const std::vector<double>& raw_signal,
        const std::vector<PeakInfo>& j_peaks
    ) {
        // 方法1: 低频带提取
        auto low_freq_result = resp_extractor.extractFromRawSignal(raw_signal);
        
        // 方法2: J波幅度调制
        auto amp_mod_result = amp_extractor.extractFromJPeakAmplitudes(j_peaks);
        
        // 共识决策
        double final_rate = resp_arbitrator.arbitrateRespirationRate(
            low_freq_result, amp_mod_result
        );
        
        return {final_rate, low_freq_result.amplitude, 0.8};
    }
};
```

## 📊 测试与验证策略

### 单元测试
- 各模块独立功能测试
- 边界条件测试
- 性能基准测试

### 集成测试  
- 使用你的实际数据进行端到端测试
- 与已有测试结果对比验证
- 不同场景下的鲁棒性测试

### 性能优化
- 内存使用优化
- 计算复杂度优化
- 实时性能调优

## 🎯 关键实现要点

1. **数据流管理**: 使用环形缓冲区确保数据不丢失
2. **多线程设计**: 各估计器可并行运行
3. **内存管理**: 避免频繁内存分配
4. **异常处理**: 完整的错误恢复机制
5. **参数调优**: 可配置的阈值和参数

## 📈 预期性能指标

- **心率精度**: ±3 bpm (40-135 bpm范围)
- **呼吸精度**: ±2 rpm (8-35 rpm范围) 
- **响应延迟**: <500ms (时域模式)
- **稳定性**: 99%可用率
- **内存占用**: <50MB
- **CPU使用**: <20% (单核)

这个实现框架完全基于你现有的算法设计，保持了所有核心创新点，并提供了详细的实现路径。建议按阶段逐步开发，每个阶段完成后进行充分测试再进入下一阶段。