package frc.robot.commands.vision;

import frc.robot.constants.VisionConstants;

public class DistanceCalculator {
    public static DoubleLookupTable powerTable = null;

    public static double getDistance(double yaw, double pitch, double area) {
        /*
        This is written in LaTeX notation.
        If you want to view this math, paste it into a LaTeX editor
         
        \text{Let $f(d)$ be the area at distance $d$ if the target is perfectly flat (not accounting for yaw and pitch)} \\
        \text{Let $p$ represent the exponent that governs the scaling behaviour of $f(d)$} \\ \\
        f(d) = 10\left(\frac{d}{\text{AREA_10_DISTANCE}}\right)^p \\ \\
        f\left(2\left(\text{AREA_10_DISTANCE}\right)\right) = \sqrt{10} \\ \\
        \sqrt{10} = 10\left(\frac{2\left(\text{AREA_10_DISTANCE}\right)}{\text{AREA_10_DISTANCE}}\right) \\ \\
        \sqrt{10} = 10\left(2\right)^p \\ \\
        \frac{\sqrt{10}}{10} = 2^p \\ \\
        10^{-\frac{1}{2}} = 2^p \\ \\
        p = \log_2(10^{-\frac{1}{2}}) \\ \\
        p = -\frac{1}{2}\left(\log_2 10\right) \\ \\
        p \approx -\frac{1}{2}(3.3219280948874) \\ \\
        p \approx -1.6609640474437 \\ \\
        \text{Now, going back to the area function $f(d)$} \\ \\
        f(d) = 10\left(\frac{d}{\text{AREA_10_DISTANCE}}\right)^p \\ \\
        \frac{f(d)}{10} = \left(\frac{d}{\text{AREA_10_DISTANCE}}\right)^p \\ \\
        \frac{d}{\text{AREA_10_DISTANCE}} = \left(\frac{f(d)}{10}\right)^{\frac{1}{p}} \\ \\
        d = \text{AREA_10_DISTANCE}\left(\frac{f(d)}{10}\right)^{\frac{1}{p}} \\ \\
        \text{Because $f(d)$ is the area at distance $d$, we can replace $f(d)$ with ``area''} \\ \\
        d = \text{AREA_10_DISTANCE}\left(\frac{\text{area}}{10}\right)^{\frac{1}{p}} \\ \\
        \text{Now, replacing p with its value} \\ \\
        d \approx \text{AREA_10_DISTANCE}\left(\frac{\text{area}}{10}\right)^{-\frac{1}{1.6609640474437}} \\ \\
        d \approx \text{AREA_10_DISTANCE}\left(\frac{\text{area}}{10}\right)^{-0.602059991328} \\ \\
        d \approx \text{AREA_10_DISTANCE}\left(\frac{10}{\text{area}}\right)^{0.602059991328}
        
        In simple terms, d = AREA_10_DISTANCE * (10/area)^0.602059991328
        */

        return VisionConstants.AREA_10_DISTANCE * Math.pow(10/area, 0.602059991328);
    }
}