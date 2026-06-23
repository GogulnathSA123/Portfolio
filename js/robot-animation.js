/* ============================================================================
   FEAR Robot: Cooperative Multi-Arm Arranging & Stacking Canvas Simulation
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

        // Stacking State Machine
        // 0: Left arm picking up cube from Left supply
        // 1: Left arm placing cube at Central transfer table
        // 2: Right arm (FEAR Robot) picking up cube from Central transfer table
        // 3: Right arm (FEAR Robot) stacking cube at Right stack area
        // 4: Reset / Clearing stack
        this.state = 0;
        this.timer = 0;
        
        // Cube states and positions
        this.cubeSize = 12;
        this.transferX = 0;
        this.transferY = 0;
        this.stackX = 0;
        
        // Array of cubes currently on the table/stack
        // Each cube: { x, y, active, color }
        this.cubes = [];
        this.currentCube = null; // Cube currently being moved
        this.stackCount = 0;

        // Initialize Robotic Arms (Two segments)
        this.armL1 = 70;
        this.armL2 = 50;

        this.leftArm = {
            baseX: 0,
            baseY: 0,
            theta1: -Math.PI / 4,
            theta2: -Math.PI / 2,
            targetTheta1: -Math.PI / 4,
            targetTheta2: -Math.PI / 2,
            gripperOpen: true
        };

        this.rightArm = { // FEAR Robot
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

        // Logical positions for task
        this.supplyX = this.leftArm.baseX - 45;
        this.supplyY = this.leftArm.baseY + 5;
        
        this.transferX = this.logicalWidth * 0.5;
        this.transferY = this.logicalHeight * 0.85; // Transfer table height

        this.stackBaseX = this.rightArm.baseX + 35;
        this.stackBaseY = this.rightArm.baseY + 5;
    }

    // Solve Inverse Kinematics for a 2-segment arm
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

        const cosElbow = (nd * nd - this.armL1 * this.armL1 - this.armL2 * this.armL2) / (2 * this.armL1 * this.armL2);
        const theta2 = flip * Math.acos(Math.max(-1, Math.min(1, cosElbow)));

        const phi1 = Math.atan2(ndy, ndx);
        const phi2 = Math.atan2(this.armL2 * Math.sin(theta2), this.armL1 + this.armL2 * Math.cos(theta2));
        const theta1 = phi1 - phi2;

        return { theta1, theta2 };
    }

    // Forward Kinematics
    solveFK(arm, baseX, baseY) {
        const x1 = baseX + this.armL1 * Math.cos(arm.theta1);
        const y1 = baseY + this.armL1 * Math.sin(arm.theta1);
        const x2 = x1 + this.armL2 * Math.cos(arm.theta1 + arm.theta2);
        const y2 = y1 + this.armL2 * Math.sin(arm.theta1 + arm.theta2);
        return { x1, y1, x2, y2 };
    }

    update() {
        this.timer++;

        // Stacking state transitions
        switch (this.state) {
            case 0: // Left Arm Pick from Supply
                // Right arm rests in ready position
                this.rightArm.gripperOpen = true;
                const rHome = this.solveIK(this.rightArm.baseX - 20, this.rightArm.baseY - 70, this.rightArm.baseX, this.rightArm.baseY, -1);
                this.rightArm.targetTheta1 = rHome.theta1;
                this.rightArm.targetTheta2 = rHome.theta2;

                // Left arm goes to supply pile
                this.leftArm.gripperOpen = true;
                const lPick = this.solveIK(this.supplyX, this.supplyY - 5, this.leftArm.baseX, this.leftArm.baseY, 1);
                this.leftArm.targetTheta1 = lPick.theta1;
                this.leftArm.targetTheta2 = lPick.theta2;

                // Grip cube
                if (this.timer === 25) {
                    this.leftArm.gripperOpen = false;
                    this.currentCube = {
                        x: this.supplyX,
                        y: this.supplyY,
                        color: this.colors.orange
                    };
                }

                if (this.timer > 35) {
                    this.state = 1;
                    this.timer = 0;
                }
                break;

            case 1: // Left Arm Place at Central Transfer
                // Left arm moves cube to transfer platform
                const lTransfer = this.solveIK(this.transferX, this.transferY - 8, this.leftArm.baseX, this.leftArm.baseY, 1);
                this.leftArm.targetTheta1 = lTransfer.theta1;
                this.leftArm.targetTheta2 = lTransfer.theta2;

                if (this.currentCube) {
                    const tip = this.solveFK(this.leftArm, this.leftArm.baseX, this.leftArm.baseY);
                    this.currentCube.x = tip.x2;
                    this.currentCube.y = tip.y2;
                }

                // Place cube
                if (this.timer === 25) {
                    this.leftArm.gripperOpen = true;
                    if (this.currentCube) {
                        this.currentCube.x = this.transferX;
                        this.currentCube.y = this.transferY - 5;
                    }
                }

                if (this.timer > 35) {
                    this.state = 2;
                    this.timer = 0;
                }
                break;

            case 2: // Right Arm (FEAR Robot) Pick from Transfer
                // Left arm retracts to resting position
                const lRest = this.solveIK(this.leftArm.baseX - 10, this.leftArm.baseY - 70, this.leftArm.baseX, this.leftArm.baseY, 1);
                this.leftArm.targetTheta1 = lRest.theta1;
                this.leftArm.targetTheta2 = lRest.theta2;

                // FEAR Robot moves to transfer table
                this.rightArm.gripperOpen = true;
                const rPick = this.solveIK(this.transferX, this.transferY - 8, this.rightArm.baseX, this.rightArm.baseY, -1);
                this.rightArm.targetTheta1 = rPick.theta1;
                this.rightArm.targetTheta2 = rPick.theta2;

                // Grip cube
                if (this.timer === 20) {
                    this.rightArm.gripperOpen = false;
                }

                if (this.timer > 30) {
                    this.state = 3;
                    this.timer = 0;
                }
                break;

            case 3: // Right Arm (FEAR Robot) Place on Stack
                // Calculate target stack height based on stackCount
                const targetY = this.stackBaseY - (this.stackCount * (this.cubeSize + 2));
                const rStack = this.solveIK(this.stackBaseX, targetY - 6, this.rightArm.baseX, this.rightArm.baseY, -1);
                this.rightArm.targetTheta1 = rStack.theta1;
                this.rightArm.targetTheta2 = rStack.theta2;

                if (this.currentCube) {
                    const tip = this.solveFK(this.rightArm, this.rightArm.baseX, this.rightArm.baseY);
                    this.currentCube.x = tip.x2;
                    this.currentCube.y = tip.y2;
                }

                // Place cube on stack
                if (this.timer === 25) {
                    this.rightArm.gripperOpen = true;
                    if (this.currentCube) {
                        this.cubes.push({
                            x: this.stackBaseX,
                            y: targetY,
                            color: this.currentCube.color
                        });
                        this.currentCube = null;
                        this.stackCount++;
                    }
                }

                if (this.timer > 35) {
                    if (this.stackCount >= 3) {
                        this.state = 4; // Clear stack if we have 3 cubes
                    } else {
                        this.state = 0; // Grab next cube
                    }
                    this.timer = 0;
                }
                break;

            case 4: // Reset / Clearing stack
                // Right arm returns to home position
                const rHome2 = this.solveIK(this.rightArm.baseX + 10, this.rightArm.baseY - 75, this.rightArm.baseX, this.rightArm.baseY, -1);
                this.rightArm.targetTheta1 = rHome2.theta1;
                this.rightArm.targetTheta2 = rHome2.theta2;

                // Wait, then fade out / clear cubes
                if (this.timer > 40) {
                    this.cubes = [];
                    this.stackCount = 0;
                    this.state = 0;
                    this.timer = 0;
                }
                break;
        }

        // Interpolate angles toward targets for smooth movement
        const lerpSpeed = 0.12;
        this.leftArm.theta1 += (this.leftArm.targetTheta1 - this.leftArm.theta1) * lerpSpeed;
        this.leftArm.theta2 += (this.leftArm.targetTheta2 - this.leftArm.theta2) * lerpSpeed;
        
        this.rightArm.theta1 += (this.rightArm.targetTheta1 - this.rightArm.theta1) * lerpSpeed;
        this.rightArm.theta2 += (this.rightArm.targetTheta2 - this.rightArm.theta2) * lerpSpeed;
    }

    drawArm(arm, baseX, baseY, accentColor) {
        const joints = this.solveFK(arm, baseX, baseY);

        // Pedestal
        this.ctx.fillStyle = this.colors.border;
        this.ctx.beginPath();
        this.ctx.arc(baseX, baseY, 14, Math.PI, 0);
        this.ctx.fill();
        this.ctx.strokeStyle = this.colors.cyan;
        this.ctx.lineWidth = 1.5;
        this.ctx.stroke();

        // Segment 1 (Shoulder to Elbow)
        this.ctx.strokeStyle = '#3F3F46';
        this.ctx.lineWidth = 8;
        this.ctx.lineCap = 'round';
        this.ctx.beginPath();
        this.ctx.moveTo(baseX, baseY);
        this.ctx.lineTo(joints.x1, joints.y1);
        this.ctx.stroke();

        this.ctx.strokeStyle = accentColor;
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        this.ctx.moveTo(baseX, baseY);
        this.ctx.lineTo(joints.x1, joints.y1);
        this.ctx.stroke();

        // Shoulder Joint
        this.ctx.fillStyle = this.colors.bg;
        this.ctx.strokeStyle = accentColor;
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        this.ctx.arc(baseX, baseY, 6, 0, 2 * Math.PI);
        this.ctx.fill();
        this.ctx.stroke();

        // Segment 2 (Elbow to Gripper)
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

        // Elbow Joint
        this.ctx.fillStyle = this.colors.bg;
        this.ctx.strokeStyle = accentColor;
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        this.ctx.arc(joints.x1, joints.y1, 5, 0, 2 * Math.PI);
        this.ctx.fill();
        this.ctx.stroke();

        // Gripper / Tool Head
        const handAngle = arm.theta1 + arm.theta2;
        const gripSize = 10;
        this.ctx.strokeStyle = accentColor;
        this.ctx.lineWidth = 2.5;

        const fAngle = arm.gripperOpen ? 0.4 : 0.12;
        
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

    drawCube(x, y, color, opacity = 1.0) {
        this.ctx.save();
        this.ctx.translate(x, y);
        this.ctx.globalAlpha = opacity;

        // Cube Fill
        this.ctx.fillStyle = color;
        this.ctx.shadowColor = color;
        this.ctx.shadowBlur = 8;
        this.ctx.fillRect(-this.cubeSize / 2, -this.cubeSize / 2, this.cubeSize, this.cubeSize);

        // Cube Border
        this.ctx.strokeStyle = '#FFFFFF';
        this.ctx.lineWidth = 1.5;
        this.ctx.shadowBlur = 0; // reset
        this.ctx.strokeRect(-this.cubeSize / 2, -this.cubeSize / 2, this.cubeSize, this.cubeSize);

        this.ctx.restore();
    }

    draw() {
        this.ctx.clearRect(0, 0, this.logicalWidth, this.logicalHeight);

        // Draw Table / Ground Support
        this.ctx.strokeStyle = this.colors.border;
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        this.ctx.moveTo(this.logicalWidth * 0.05, this.logicalHeight * 0.86);
        this.ctx.lineTo(this.logicalWidth * 0.95, this.logicalHeight * 0.86);
        this.ctx.stroke();

        // Draw Central Transfer Platform
        this.ctx.fillStyle = '#18181B';
        this.ctx.fillRect(this.transferX - 25, this.transferY, 50, 6);
        this.ctx.strokeStyle = this.colors.border;
        this.ctx.lineWidth = 1.5;
        this.ctx.strokeRect(this.transferX - 25, this.transferY, 50, 6);

        // Draw Supply Pile Box (Left side)
        this.ctx.fillStyle = '#18181B';
        this.ctx.fillRect(this.supplyX - 15, this.supplyY + 3, 30, 4);
        this.ctx.strokeStyle = this.colors.border;
        this.ctx.strokeRect(this.supplyX - 15, this.supplyY + 3, 30, 4);

        // Draw Left Arm (Arranging Arm - Cyan)
        this.drawArm(this.leftArm, this.leftArm.baseX, this.leftArm.baseY, this.colors.cyan);

        // Draw Right Arm (FEAR Robot - Purple)
        this.drawArm(this.rightArm, this.rightArm.baseX, this.rightArm.baseY, this.colors.purple);

        // Draw Stacking area box (Right side)
        this.ctx.fillStyle = '#18181B';
        this.ctx.fillRect(this.stackBaseX - 15, this.stackBaseY + 3, 30, 4);
        this.ctx.strokeStyle = this.colors.border;
        this.ctx.strokeRect(this.stackBaseX - 15, this.stackBaseY + 3, 30, 4);

        // Draw existing cubes in stack
        this.cubes.forEach((cube) => {
            let opacity = 1.0;
            if (this.state === 4) {
                // Fade out stack on reset
                opacity = Math.max(0, 1.0 - (this.timer / 40));
            }
            this.drawCube(cube.x, cube.y, cube.color, opacity);
        });

        // Draw current cube being moved
        if (this.currentCube) {
            this.drawCube(this.currentCube.x, this.currentCube.y, this.currentCube.color);
        }

        // Draw supply cube waiting if Left Arm is resting
        if (this.state === 2 || this.state === 3 || this.state === 4) {
            this.drawCube(this.supplyX, this.supplyY, this.colors.orange);
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
