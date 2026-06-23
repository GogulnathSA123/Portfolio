/* ============================================================================
   Robotic Arm Throw-and-Catch Canvas Simulation
   ============================================================================ */
class RobotAnimation {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        if (!this.canvas) return;
        this.ctx = this.canvas.getContext('2d');
        this.width = this.canvas.width;
        this.height = this.canvas.height;

        // Colors matching design system
        this.colors = {
            cyan: '#00F0FF',
            purple: '#8A2BE2',
            green: '#00FF88',
            orange: '#FF6B35',
            bg: '#111827',
            border: '#27272A',
            text: '#E4E4E7'
        };

        // State Machine
        // 0: Windup (Left arm preparing)
        // 1: Throw swing (Left arm releasing cube)
        // 2: In Flight (Cube traveling, right arm tracking)
        // 3: Catch & Recoil (Right arm catching and absorbing impact)
        // 4: Return & Reset (Right arm returns to home, fades out cube)
        this.state = 0;
        this.timer = 0;

        // Physics/Trajectory parameters
        this.gravity = 0.15;
        this.cube = {
            x: 0,
            y: 0,
            vx: 0,
            vy: 0,
            size: 10,
            rotation: 0,
            vRot: 0.1,
            trail: []
        };

        // Initialize Robotic Arms
        // Segment lengths
        this.armL1 = 70;
        this.armL2 = 50;

        this.leftArm = {
            baseX: 0,
            baseY: 0,
            theta1: -Math.PI / 4,
            theta2: -Math.PI / 2,
            targetTheta1: -Math.PI / 4,
            targetTheta2: -Math.PI / 2,
            gripperOpen: false
        };

        this.rightArm = {
            baseX: 0,
            baseY: 0,
            theta1: -3 * Math.PI / 4,
            theta2: -Math.PI / 2,
            targetTheta1: -3 * Math.PI / 4,
            targetTheta2: -Math.PI / 2,
            gripperOpen: true
        };

        this.initSize();
        window.addEventListener('resize', () => this.initSize());
        this.loop();
    }

    initSize() {
        const rect = this.canvas.getBoundingClientRect();
        this.width = rect.width * window.devicePixelRatio;
        this.height = rect.height * window.devicePixelRatio;
        this.canvas.width = this.width;
        this.canvas.height = this.height;
        this.ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
        
        // Reset logical dimensions
        this.logicalWidth = rect.width;
        this.logicalHeight = rect.height;

        // Base positions relative to canvas size
        this.leftArm.baseX = this.logicalWidth * 0.18;
        this.leftArm.baseY = this.logicalHeight * 0.85;

        this.rightArm.baseX = this.logicalWidth * 0.82;
        this.rightArm.baseY = this.logicalHeight * 0.85;
    }

    // Solve Inverse Kinematics for a 2-segment arm
    // Given target (tx, ty) and base (bx, by), returns segment angles {theta1, theta2}
    solveIK(tx, ty, bx, by, flip = 1) {
        const dx = tx - bx;
        const dy = ty - by;
        const d = Math.sqrt(dx * dx + dy * dy);

        // Clamp target to maximum reach
        const maxReach = (this.armL1 + this.armL2) * 0.99;
        let targetX = tx;
        let targetY = ty;
        if (d > maxReach) {
            const angle = Math.atan2(dy, dx);
            targetX = bx + Math.cos(angle) * maxReach;
            targetY = by + Math.sin(angle) * maxReach;
        }

        const ndx = targetX - bx;
        const ndy = targetY - by;
        const nd = Math.sqrt(ndx * ndx + ndy * ndy);

        // Law of Cosines for Elbow Angle
        const cosElbow = (nd * nd - this.armL1 * this.armL1 - this.armL2 * this.armL2) / (2 * this.armL1 * this.armL2);
        const theta2 = flip * Math.acos(Math.max(-1, Math.min(1, cosElbow)));

        // Shoulder Angle
        const phi1 = Math.atan2(ndy, ndx);
        const phi2 = Math.atan2(this.armL2 * Math.sin(theta2), this.armL1 + this.armL2 * Math.cos(theta2));
        const theta1 = phi1 - phi2;

        return { theta1, theta2 };
    }

    // Forward Kinematics to find gripper tip position
    solveFK(arm, baseX, baseY) {
        const x1 = baseX + this.armL1 * Math.cos(arm.theta1);
        const y1 = baseY + this.armL1 * Math.sin(arm.theta1);
        const x2 = x1 + this.armL2 * Math.cos(arm.theta1 + arm.theta2);
        const y2 = y1 + this.armL2 * Math.sin(arm.theta1 + arm.theta2);
        return { x1, y1, x2, y2 };
    }

    update() {
        this.timer++;

        // State Transitions and Arm Targets
        switch (this.state) {
            case 0: // Windup (Preparatory phase)
                // Left arm winds back
                this.leftArm.gripperOpen = false;
                const windTargetX = this.leftArm.baseX - 40;
                const windTargetY = this.leftArm.baseY - 50;
                const windAngles = this.solveIK(windTargetX, windTargetY, this.leftArm.baseX, this.leftArm.baseY, 1);
                this.leftArm.targetTheta1 = windAngles.theta1;
                this.leftArm.targetTheta2 = windAngles.theta2;

                // Right arm waits in ready position
                this.rightArm.gripperOpen = true;
                const rightReadyX = this.rightArm.baseX - 30;
                const rightReadyY = this.rightArm.baseY - 70;
                const rightAngles = this.solveIK(rightReadyX, rightReadyY, this.rightArm.baseX, this.rightArm.baseY, -1);
                this.rightArm.targetTheta1 = rightAngles.theta1;
                this.rightArm.targetTheta2 = rightAngles.theta2;

                if (this.timer > 50) {
                    this.state = 1;
                    this.timer = 0;
                }
                break;

            case 1: // Throw swing
                // Left arm swings forward rapidly
                const throwTargetX = this.leftArm.baseX + 50;
                const throwTargetY = this.leftArm.baseY - 110;
                const throwAngles = this.solveIK(throwTargetX, throwTargetY, this.leftArm.baseX, this.leftArm.baseY, 1);
                this.leftArm.targetTheta1 = throwAngles.theta1;
                this.leftArm.targetTheta2 = throwAngles.theta2;

                // Release point
                if (this.timer === 15) {
                    const tip = this.solveFK(this.leftArm, this.leftArm.baseX, this.leftArm.baseY);
                    this.cube.x = tip.x2;
                    this.cube.y = tip.y2;
                    
                    // Dynamic launch velocity based on target distance
                    const dx = (this.rightArm.baseX - 50) - tip.x2;
                    const dy = (this.rightArm.baseY - 120) - tip.y2;
                    const flightTime = 40;
                    
                    this.cube.vx = dx / flightTime;
                    this.cube.vy = (dy - 0.5 * this.gravity * flightTime * flightTime) / flightTime;
                    this.cube.trail = [];
                    this.cube.rotation = 0;
                    this.cube.vRot = 0.15;
                    this.leftArm.gripperOpen = true; // Open gripper to release
                }

                if (this.timer > 20) {
                    this.state = 2;
                    this.timer = 0;
                }
                break;

            case 2: // In Flight
                // Apply Gravity to Cube
                this.cube.x += this.cube.vx;
                this.cube.y += this.cube.vy;
                this.cube.vy += this.gravity;
                this.cube.rotation += this.cube.vRot;

                // Save trail
                this.cube.trail.push({ x: this.cube.x, y: this.cube.y });
                if (this.cube.trail.length > 15) this.cube.trail.shift();

                // Left arm returns to resting position
                const leftRestX = this.leftArm.baseX - 10;
                const leftRestY = this.leftArm.baseY - 80;
                const leftRestAngles = this.solveIK(leftRestX, leftRestY, this.leftArm.baseX, this.leftArm.baseY, 1);
                this.leftArm.targetTheta1 = leftRestAngles.theta1;
                this.leftArm.targetTheta2 = leftRestAngles.theta2;

                // Right arm tracks the incoming cube
                // Gripper tracks the cube's horizontal trajectory but ready to intercept
                const rightTrackX = Math.max(this.leftArm.baseX + 100, Math.min(this.cube.x, this.rightArm.baseX));
                const rightTrackY = Math.min(this.cube.y, this.rightArm.baseY - 50);
                const rightTrackAngles = this.solveIK(rightTrackX, rightTrackY, this.rightArm.baseX, this.rightArm.baseY, -1);
                this.rightArm.targetTheta1 = rightTrackAngles.theta1;
                this.rightArm.targetTheta2 = rightTrackAngles.theta2;

                // Check for catch (proximity to right gripper tip)
                const rightTip = this.solveFK(this.rightArm, this.rightArm.baseX, this.rightArm.baseY);
                const catchDist = Math.hypot(this.cube.x - rightTip.x2, this.cube.y - rightTip.y2);

                if (catchDist < 15 || this.cube.x >= rightTip.x2) {
                    this.cube.x = rightTip.x2;
                    this.cube.y = rightTip.y2;
                    this.rightArm.gripperOpen = false; // Close gripper to catch
                    this.state = 3;
                    this.timer = 0;
                }
                break;

            case 3: // Catch & Recoil
                // Move cube with right arm tip
                const rightTipPos = this.solveFK(this.rightArm, this.rightArm.baseX, this.rightArm.baseY);
                this.cube.x = rightTipPos.x2;
                this.cube.y = rightTipPos.y2;

                // Recoil absorption position (absorb momentum by pulling back)
                const recoilX = this.rightArm.baseX + 30;
                const recoilY = this.rightArm.baseY - 40;
                const recoilAngles = this.solveIK(recoilX, recoilY, this.rightArm.baseX, this.rightArm.baseY, -1);
                this.rightArm.targetTheta1 = recoilAngles.theta1;
                this.rightArm.targetTheta2 = recoilAngles.theta2;

                if (this.timer > 15) {
                    this.state = 4;
                    this.timer = 0;
                }
                break;

            case 4: // Return & Reset
                // Move cube with right arm tip
                const rightTipPos2 = this.solveFK(this.rightArm, this.rightArm.baseX, this.rightArm.baseY);
                this.cube.x = rightTipPos2.x2;
                this.cube.y = rightTipPos2.y2;

                // Return to home position
                const rightHomeX = this.rightArm.baseX + 10;
                const rightHomeY = this.rightArm.baseY - 80;
                const rightHomeAngles = this.solveIK(rightHomeX, rightHomeY, this.rightArm.baseX, this.rightArm.baseY, -1);
                this.rightArm.targetTheta1 = rightHomeAngles.theta1;
                this.rightArm.targetTheta2 = rightHomeAngles.theta2;

                // Left arm moves to rest/grab next cube
                const leftReadyX2 = this.leftArm.baseX - 20;
                const leftReadyY2 = this.leftArm.baseY - 60;
                const leftReadyAngles2 = this.solveIK(leftReadyX2, leftReadyY2, this.leftArm.baseX, this.leftArm.baseY, 1);
                this.leftArm.targetTheta1 = leftReadyAngles2.theta1;
                this.leftArm.targetTheta2 = leftReadyAngles2.theta2;

                if (this.timer > 30) {
                    this.state = 0;
                    this.timer = 0;
                }
                break;
        }

        // Interpolate arm angles toward target for smooth organic movement
        const lerpSpeed = 0.12;
        this.leftArm.theta1 += (this.leftArm.targetTheta1 - this.leftArm.theta1) * lerpSpeed;
        this.leftArm.theta2 += (this.leftArm.targetTheta2 - this.leftArm.theta2) * lerpSpeed;
        
        const rightLerpSpeed = this.state === 2 ? 0.25 : 0.12; // Track faster in flight
        this.rightArm.theta1 += (this.rightArm.targetTheta1 - this.rightArm.theta1) * rightLerpSpeed;
        this.rightArm.theta2 += (this.rightArm.targetTheta2 - this.rightArm.theta2) * rightLerpSpeed;
    }

    drawArm(arm, baseX, baseY, accentColor) {
        const joints = this.solveFK(arm, baseX, baseY);

        // Draw Base Joint pedestal
        this.ctx.fillStyle = this.colors.border;
        this.ctx.beginPath();
        this.ctx.arc(baseX, baseY, 14, Math.PI, 0);
        this.ctx.fill();
        this.ctx.strokeStyle = this.colors.cyan;
        this.ctx.lineWidth = 1.5;
        this.ctx.stroke();

        // Draw Segment 1 (Shoulder to Elbow)
        this.ctx.strokeStyle = '#3F3F46';
        this.ctx.lineWidth = 8;
        this.ctx.lineCap = 'round';
        this.ctx.beginPath();
        this.ctx.moveTo(baseX, baseY);
        this.ctx.lineTo(joints.x1, joints.y1);
        this.ctx.stroke();

        // Inner Segment 1 detailing (wireframe glow)
        this.ctx.strokeStyle = accentColor;
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        this.ctx.moveTo(baseX, baseY);
        this.ctx.lineTo(joints.x1, joints.y1);
        this.ctx.stroke();

        // Draw Shoulder Joint
        this.ctx.fillStyle = this.colors.bg;
        this.ctx.strokeStyle = accentColor;
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        this.ctx.arc(baseX, baseY, 6, 0, 2 * Math.PI);
        this.ctx.fill();
        this.ctx.stroke();

        // Draw Segment 2 (Elbow to Gripper)
        this.ctx.strokeStyle = '#3F3F46';
        this.ctx.lineWidth = 6;
        this.ctx.beginPath();
        this.ctx.moveTo(joints.x1, joints.y1);
        this.ctx.lineTo(joints.x2, joints.y2);
        this.ctx.stroke();

        this.ctx.strokeStyle = '#FFFFFF';
        this.ctx.lineWidth = 1.5;
        this.ctx.beginPath();
        this.ctx.moveTo(joints.x1, joints.y1);
        this.ctx.lineTo(joints.x2, joints.y2);
        this.ctx.stroke();

        // Draw Elbow Joint
        this.ctx.fillStyle = this.colors.bg;
        this.ctx.strokeStyle = accentColor;
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        this.ctx.arc(joints.x1, joints.y1, 5, 0, 2 * Math.PI);
        this.ctx.fill();
        this.ctx.stroke();

        // Draw Gripper / Tool Head
        const handAngle = arm.theta1 + arm.theta2;
        const gripSize = 10;
        this.ctx.strokeStyle = accentColor;
        this.ctx.lineWidth = 2.5;

        // Draw two fingers
        const fAngle = arm.gripperOpen ? 0.4 : 0.15;
        
        // Upper finger
        this.ctx.beginPath();
        this.ctx.moveTo(joints.x2, joints.y2);
        this.ctx.lineTo(
            joints.x2 + gripSize * Math.cos(handAngle - fAngle),
            joints.y2 + gripSize * Math.sin(handAngle - fAngle)
        );
        this.ctx.stroke();

        // Lower finger
        this.ctx.beginPath();
        this.ctx.moveTo(joints.x2, joints.y2);
        this.ctx.lineTo(
            joints.x2 + gripSize * Math.cos(handAngle + fAngle),
            joints.y2 + gripSize * Math.sin(handAngle + fAngle)
        );
        this.ctx.stroke();
    }

    draw() {
        this.ctx.clearRect(0, 0, this.logicalWidth, this.logicalHeight);


        // Draw Trajectory Parabolic Guide (dotted)
        if (this.state === 2) {
            this.ctx.strokeStyle = 'rgba(0, 240, 255, 0.15)';
            this.ctx.lineWidth = 1;
            this.ctx.setLineDash([4, 4]);
            this.ctx.beginPath();
            this.ctx.moveTo(this.leftArm.baseX + 30, this.leftArm.baseY - 90);
            this.ctx.quadraticCurveTo(
                this.logicalWidth * 0.5,
                this.logicalHeight * 0.1,
                this.rightArm.baseX - 30,
                this.rightArm.baseY - 70
            );
            this.ctx.stroke();
            this.ctx.setLineDash([]); // Reset
        }

        // Draw Motion Trail of Cube
        if (this.state === 2 && this.cube.trail.length > 1) {
            this.ctx.beginPath();
            this.ctx.moveTo(this.cube.trail[0].x, this.cube.trail[0].y);
            for (let i = 1; i < this.cube.trail.length; i++) {
                this.ctx.lineTo(this.cube.trail[i].x, this.cube.trail[i].y);
            }
            this.ctx.strokeStyle = 'rgba(0, 255, 136, 0.3)';
            this.ctx.lineWidth = 4;
            this.ctx.lineCap = 'round';
            this.ctx.lineJoin = 'round';
            this.ctx.stroke();

            this.ctx.strokeStyle = 'rgba(0, 240, 255, 0.6)';
            this.ctx.lineWidth = 1.5;
            this.ctx.stroke();
        }

        // Draw Left Arm (Thrower, Cyan accent)
        this.drawArm(this.leftArm, this.leftArm.baseX, this.leftArm.baseY, this.colors.cyan);

        // Draw Right Arm (Catcher, Purple/Orange accent)
        this.drawArm(this.rightArm, this.rightArm.baseX, this.rightArm.baseY, this.colors.purple);

        // Draw Cube
        if (this.state === 2 || this.state === 3 || (this.state === 4 && this.timer < 20)) {
            this.ctx.save();
            this.ctx.translate(this.cube.x, this.cube.y);
            this.ctx.rotate(this.cube.rotation);

            // Draw glowing cube
            this.ctx.fillStyle = this.colors.orange;
            this.ctx.shadowColor = this.colors.orange;
            this.ctx.shadowBlur = 10;
            this.ctx.fillRect(-this.cube.size / 2, -this.cube.size / 2, this.cube.size, this.cube.size);

            this.ctx.strokeStyle = '#FFFFFF';
            this.ctx.lineWidth = 1.5;
            this.ctx.shadowBlur = 0; // reset
            this.ctx.strokeRect(-this.cube.size / 2, -this.cube.size / 2, this.cube.size, this.cube.size);

            this.ctx.restore();
        } else if (this.state === 0) {
            // Draw next cube loading in left gripper tip
            const leftTip = this.solveFK(this.leftArm, this.leftArm.baseX, this.leftArm.baseY);
            this.ctx.save();
            this.ctx.translate(leftTip.x2, leftTip.y2);
            // Slowly pulsing/glowing loader cube
            const scale = 0.5 + 0.3 * Math.abs(Math.sin(this.timer * 0.08));
            this.ctx.fillStyle = this.colors.orange;
            this.ctx.globalAlpha = 0.8;
            this.ctx.fillRect(-this.cube.size * scale / 2, -this.cube.size * scale / 2, this.cube.size * scale, this.cube.size * scale);
            this.ctx.restore();
        }
    }

    loop() {
        this.update();
        this.draw();
        requestAnimationFrame(() => this.loop());
    }
}

// Instantiate on load
document.addEventListener('DOMContentLoaded', () => {
    new RobotAnimation('robotCanvas');
});
