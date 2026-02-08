# FTC Black Ice Path Follower 项目分析

## 项目简介 (Introduction)

本项目名为 **Black Ice Path Follower**，是一个为 FTC (FIRST Tech Challenge) 机器人设计的先进路径跟随系统。该项目主要用于 2025 赛季的休赛期实验，旨在探索曲线跟随（Curve Following）和自定义减速曲线（Deceleration Profiles）的实现。

其核心目标是超越传统的 PID 控制，通过引入更符合物理规律的控制算法，实现机器人高速运动下的精准控制和快速停稳。

## 目的与意义 (Purpose & Significance)

1.  **解决非线性刹车问题**：传统的 PID 控制器假设系统的响应是线性的。然而，直流电机在反向制动（Back-EMF Braking）或零功率刹车（Zero Power Brake）时表现出强烈的非线性特征。本项目通过引入“二次阻尼”（Quadratic Damping）项来模拟这种物理特性。
2.  **提升路径跟随精度**：通过矢量控制（Vector Control），将驱动力分解为向心力、切向驱动力、平移校正力和转向力，实现了对贝塞尔曲线（Bézier Curves）等复杂路径的流畅跟随。
3.  **优化减速性能**：利用动量补偿（Momentum Compensation）和前馈控制（Feedforward），实现了比传统 PID 快约 5 倍的减速响应，允许机器人在高速状态下更晚开始刹车，从而缩短整体路径运行时间。

## 数学原理 (Mathematical Principles)

### 1. 二次阻尼 PID 控制器 (Quadratic Damped PID Controller)

这是本项目的核心创新点。传统的 PID 控制器公式为：

$$ u(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt} $$

而在本项目中，为了模拟空气阻力或电机反电动势带来的非线性阻尼，引入了一个与速度平方成正比的阻尼项。虽然代码实现上略有不同，但其物理本质可以描述为：

$$ u(t) = K_p e(t) - K_d v(t) - K_q v(t) |v(t)| $$

在代码 `QuadraticDampedPIDController.java` 中，实现方式是通过修改误差项来引入预测性的阻尼：

$$ \text{predictedBrakingDisplacement} = v \cdot |v| \cdot k_{\text{Friction}} + v \cdot k_{\text{Braking}} $$

然后将修正后的误差传递给标准 PD 控制器：

$$ u = \text{super.runFromError}(e - \text{predictedBrakingDisplacement}, -v) $$

展开后，输出功率 $u$ 近似为：

$$ u \approx K_p (e - (k_Q v |v| + k_B v)) + K_d (-v) $$

其中：
- $k_Q$ ($k_{\text{Friction}}$) 对应二次阻尼系数，模拟高速下的非线性摩擦/反电动势。
- $k_B$ ($k_{\text{Braking}}$) 对应线性阻尼系数。
- 该项实际上是在预测“如果现在开始刹车，机器人会滑行多远”，并提前进行补偿。

### 2. 动量补偿与前馈控制 (Momentum Compensation)

在 `DrivePowerController.java` 中，项目利用运动学公式计算动量补偿。

根据匀变速直线运动公式 $v_f^2 = v_i^2 + 2ad$，若机器人以自然减速率 $a_{dec}$ 滑行，其停止距离或末速度可以预测。

代码中计算了“滑行末速度” ($v_{coast}$)：

$$ v_{coast} = \sqrt{v_{current}^2 - 2 \cdot a_{dec} \cdot d_{remaining}} $$

(注意：代码中使用了 `Math.abs` 处理根号内为负的情况，即 $2 a_{dec} d > v^2$，此时意味着机器人会在到达终点前停下)。

进而计算“因动量损失的速度” ($\Delta v_{loss}$)：

$$ \Delta v_{loss} = v_{current} - v_{coast} $$

最后调整目标速度的前馈量：

$$ v_{feedforward} = v_{target} - \Delta v_{loss} $$

这一机制确保了控制器能够感知当前的动量，在需要减速时自动降低前馈输出，利用机器人的自然惯性滑行至目标点，从而节省能量并减少超调。

## 控制原理 (Control Principles)

### 矢量优先级控制 (Vector Prioritization)

在 `DrivePowerController.java` 中，机器人最终的驱动向量是由多个分量按优先级合成的。这种方法确保了关键的物理约束（如保持在路径上）优先于次要目标（如达到目标速度）。

优先级如下：

1.  **向心力 (Centripetal Force)**:
    $$ F_c = m \frac{v^2}{r} $$
    代码实现：`tangentialVelocity^2 * curvature * centripetalScaling`。
    方向：垂直于路径切线指向圆心。
    目的：提供转弯所需的向心力，防止机器人在曲线运动中甩出路径。

2.  **平移校正 (Translational Correction)**:
    利用 PID 控制器修正横向误差（Cross-track error）。
    方向：垂直于路径切线。
    目的：将机器人拉回路径中心线。

3.  **航向校正 (Heading Correction)**:
    利用 PID 控制器修正角度误差。
    目的：确保机器人车头朝向正确。

4.  **驱动动力 (Drive Power)**:
    切向驱动力，由速度控制器（Velocity Controller）计算。
    方向：沿路径切线方向。
    目的：控制机器人的前进速度。

### 剩余功率分配

为了防止电机饱和（输出超过 1.0），控制器采用“剩余功率”机制。高优先级的力先占用功率，低优先级的力只能使用剩余的功率预算。

$$ P_{remaining}^2 = 1.0 - P_{centripetal}^2 - P_{translational}^2 - \dots $$

这确保了在高速转弯等极限情况下，机器人会优先保证不脱轨（向心力），而不是盲目加速。

## 总结

Black Ice Path Follower 项目展示了如何将物理模型（运动学方程、阻尼模型）深度融合到机器人控制算法中。通过二次阻尼 PID 和动量补偿，它突破了传统线性控制器的局限，为竞技机器人提供了一个高效、精准且响应迅速的底层控制框架。
