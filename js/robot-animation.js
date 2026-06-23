class RobotAnimation {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        if (!this.canvas) return;
        this.ctx = this.canvas.getContext('2d');

        // Animation Settings
        this.armL1 = 44; // Length of upper arm
        this.armL2 = 38; // Length of forearm
        this.cubeSize = 16;
        
        this.colors = {
            cyan: '#00F0FF',
            purple: '#A855F7',
            orange: '#F97316',
            bg: '#0A0A0A',
            border: '#27272A',
            grid: '#1F2937'
        };

        // Robot mobile base and state parameters
        this.robotX = 250;      // Start at center
        this.wheelAngle = 0;
        this.state = 0;          // 0: Driving to supply, 1: Picking, 2: Driving to center & hand-off, 3: Driving to stack, 4: Placing, 5: Reset stack
        this.timer = 0;
        this.stackCount = 0;
        this.cubes = [];
        this.currentCube = null;
        this.particles = [];    // For movement dust effect

        // Left and Right Arms (2 DOF manipulators)
        this.leftArm = {
            theta1: -Math.PI / 4,
            theta2: -Math.PI / 2,
            targetTheta1: -Math.PI / 4,
            targetTheta2: -Math.PI / 2,
            gripperOpen: true
        };

        this.rightArm = {
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
        
        this.logicalWidth = rect.width;
        this.logicalHeight = rect.height;

        // Base task position markers
        this.supplyX = 60;
        this.stackX = this.logicalWidth - 60;

        // Robot driving endpoints
        this.pickupRobotX = 115;
        this.dropRobotX = this.logicalWidth - 115;
        this.centerRobotX = this.logicalWidth * 0.5;

        // Adjust coordinates for floor lines
        this.floorY = this.logicalHeight * 0.82;
        this.supplyY = this.floorY - 8;
        this.stackBaseY = this.floorY - 8;

        if (this.robotX === 250) {
            this.robotX = this.centerRobotX;
        }
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

    setArmTargets(ltx, lty, rtx, rty) {
        const baseY = this.floorY - 43;
        const baseLeftX = this.robotX - 18;
        const baseLeftY = baseY + 4;
        const baseRightX = this.robotX + 18;
        const baseRightY = baseY + 4;

        if (ltx !== null && lty !== null) {
            const lIK = this.solveIK(ltx, lty, baseLeftX, baseLeftY, 1);
            this.leftArm.targetTheta1 = lIK.theta1;
            this.leftArm.targetTheta2 = lIK.theta2;
        }
        if (rtx !== null && rty !== null) {
            const rIK = this.solveIK(rtx, rty, baseRightX, baseRightY, -1);
            this.rightArm.targetTheta1 = rIK.theta1;
            this.rightArm.targetTheta2 = rIK.theta2;
        }
    }

    createDustParticles() {
        if (Math.random() < 0.3) {
            const wheelOffset = Math.random() > 0.5 ? -20 : 20;
            this.particles.push({
                x: this.robotX + wheelOffset + (Math.random() * 6 - 3),
                y: this.floorY,
                vx: -Math.sign(this.pickupRobotX - this.robotX) * (0.5 + Math.random()),
                vy: -Math.random() * 0.5,
                size: Math.random() * 3 + 1,
                alpha: 0.8,
                life: 0,
                maxLife: 30 + Math.random() * 20
            });
        }
    }

    updateParticles() {
        for (let i = this.particles.length - 1; i >= 0; i--) {
            const p = this.particles[i];
            p.x += p.vx;
            p.y += p.vy;
            p.alpha = 1.0 - (p.life / p.maxLife);
            p.life++;
            if (p.life >= p.maxLife) {
                this.particles.splice(i, 1);
            }
        }
    }

    update() {
        this.timer++;
        this.updateParticles();

        const baseY = this.floorY - 43;
        const baseLeftX = this.robotX - 18;
        const baseLeftY = baseY + 4;
        const baseRightX = this.robotX + 18;
        const baseRightY = baseY + 4;

        // Arm target positions relative to mobile chassis
        const rTransitX = this.robotX + 15;
        const rTransitY = baseY - 40;
        const lTransitX = this.robotX - 15;
        const lTransitY = baseY - 40;

        const rRestX = this.robotX + 25;
        const rRestY = baseY - 20;
        const lRestX = this.robotX - 25;
        const lRestY = baseY - 20;

        const driveSpeed = 2.0;

        switch (this.state) {
            case 0: // Drive to Supply (Left)
                const dx0 = this.pickupRobotX - this.robotX;
                if (Math.abs(dx0) > 2) {
                    const step = Math.sign(dx0) * driveSpeed;
                    this.robotX += step;
                    this.wheelAngle += step / 12;
                    this.createDustParticles();
                } else {
                    this.robotX = this.pickupRobotX;
                    if (this.timer > 15) {
                        this.state = 1;
                        this.timer = 0;
                    }
                }

                // Keep arms in compact transit/resting positions
                this.rightArm.gripperOpen = true;
                this.leftArm.gripperOpen = true;
                this.setArmTargets(lRestX, lRestY, rRestX, rRestY);
                break;

            case 1: // Pick Cube from Supply (Right arm picks)
                this.leftArm.gripperOpen = true;
                this.setArmTargets(lRestX, lRestY, null, null);

                if (this.timer < 25) {
                    // Right arm descends to supply cube
                    this.rightArm.gripperOpen = true;
                    this.setArmTargets(null, null, this.supplyX, this.supplyY - 4);
                } else if (this.timer < 40) {
                    // Close gripper
                    this.rightArm.gripperOpen = false;
                    if (!this.currentCube) {
                        this.currentCube = {
                            x: this.supplyX,
                            y: this.supplyY,
                            color: this.colors.orange,
                            heldBy: 'right'
                        };
                    }
                } else if (this.timer < 60) {
                    // Lift right arm
                    this.setArmTargets(null, null, rTransitX, rTransitY);
                } else {
                    this.state = 2;
                    this.timer = 0;
                }
                break;

            case 2: // Drive to Center and Hand-off
                const dx2 = this.centerRobotX - this.robotX;
                if (Math.abs(dx2) > 2) {
                    const step = Math.sign(dx2) * driveSpeed;
                    this.robotX += step;
                    this.wheelAngle += step / 12;
                    this.createDustParticles();
                    
                    // Maintain cube in right arm transit position
                    this.setArmTargets(lRestX, lRestY, rTransitX, rTransitY);
                } else {
                    this.robotX = this.centerRobotX;
                    
                    // Hand-off coordinates
                    const handoffX = this.robotX;
                    const handoffY = baseY - 45;

                    if (this.timer < 25) {
                        // Move both arms to the meeting hand-off point
                        this.leftArm.gripperOpen = true;
                        this.setArmTargets(handoffX, handoffY, handoffX, handoffY);
                    } else if (this.timer < 40) {
                        // Left arm grips the cube
                        this.leftArm.gripperOpen = false;
                        if (this.currentCube) {
                            this.currentCube.heldBy = 'left';
                        }
                    } else if (this.timer < 55) {
                        // Right arm releases gripper
                        this.rightArm.gripperOpen = true;
                    } else if (this.timer < 75) {
                        // Retract right arm, keep left arm in transit hold
                        this.setArmTargets(lTransitX, lTransitY, rRestX, rRestY);
                    } else {
                        this.state = 3;
                        this.timer = 0;
                    }
                }
                break;

            case 3: // Drive to Stack (Right)
                const dx3 = this.dropRobotX - this.robotX;
                if (Math.abs(dx3) > 2) {
                    const step = Math.sign(dx3) * driveSpeed;
                    this.robotX += step;
                    this.wheelAngle += step / 12;
                    this.createDustParticles();

                    // Left arm holds, right arm rests
                    this.setArmTargets(lTransitX, lTransitY, rRestX, rRestY);
                } else {
                    this.robotX = this.dropRobotX;
                    if (this.timer > 15) {
                        this.state = 4;
                        this.timer = 0;
                    }
                }
                break;

            case 4: // Place Cube on Stack (Left arm places)
                this.rightArm.gripperOpen = true;
                this.setArmTargets(null, null, rRestX, rRestY);

                const targetY = this.stackBaseY - (this.stackCount * (this.cubeSize + 2));

                if (this.timer < 25) {
                    // Descend to stack position
                    this.leftArm.gripperOpen = false;
                    this.setArmTargets(this.stackX, targetY - 4, null, null);
                } else if (this.timer < 45) {
                    // Release cube
                    this.leftArm.gripperOpen = true;
                    if (this.currentCube) {
                        this.cubes.push({
                            x: this.stackX,
                            y: targetY,
                            color: this.currentCube.color
                        });
                        this.currentCube = null;
                        this.stackCount++;
                    }
                } else if (this.timer < 65) {
                    // Retract left arm to rest
                    this.setArmTargets(lRestX, lRestY, null, null);
                } else {
                    this.state = 5;
                    this.timer = 0;
                }
                break;

            case 5: // Check Stack & Reset
                this.setArmTargets(lRestX, lRestY, rRestX, rRestY);
                if (this.stackCount >= 3) {
                    if (this.timer > 45) {
                        this.cubes = [];
                        this.stackCount = 0;
                        this.state = 0;
                        this.timer = 0;
                    }
                } else {
                    this.state = 0;
                    this.timer = 0;
                }
                break;
        }

        // Keep current cube attached to the correct gripper
        if (this.currentCube) {
            if (this.currentCube.heldBy === 'right') {
                const joints = this.solveFK(this.rightArm, baseRightX, baseRightY);
                this.currentCube.x = joints.x2;
                this.currentCube.y = joints.y2;
            } else {
                const joints = this.solveFK(this.leftArm, baseLeftX, baseLeftY);
                this.currentCube.x = joints.x2;
                this.currentCube.y = joints.y2;
            }
        }

        // Smoothly interpolate angles
        const lerpSpeed = 0.15;
        this.leftArm.theta1 += (this.leftArm.targetTheta1 - this.leftArm.theta1) * lerpSpeed;
        this.leftArm.theta2 += (this.leftArm.targetTheta2 - this.leftArm.theta2) * lerpSpeed;
        this.rightArm.theta1 += (this.rightArm.targetTheta1 - this.rightArm.theta1) * lerpSpeed;
        this.rightArm.theta2 += (this.rightArm.targetTheta2 - this.rightArm.theta2) * lerpSpeed;
    }

    drawWheel(cx, cy, r, angle) {
        this.ctx.save();
        this.ctx.translate(cx, cy);
        this.ctx.rotate(angle);

        // Outer Tire
        this.ctx.fillStyle = '#1C1917';
        this.ctx.beginPath();
        this.ctx.arc(0, 0, r, 0, Math.PI * 2);
        this.ctx.fill();

        // Rim border
        this.ctx.strokeStyle = '#57534E';
        this.ctx.lineWidth = 1.5;
        this.ctx.beginPath();
        this.ctx.arc(0, 0, r - 3, 0, Math.PI * 2);
        this.ctx.stroke();

        // Spokes
        this.ctx.strokeStyle = this.colors.cyan;
        this.ctx.lineWidth = 1;
        for (let i = 0; i < 4; i++) {
            const spAngle = (i * Math.PI) / 2;
            this.ctx.beginPath();
            this.ctx.moveTo(0, 0);
            this.ctx.lineTo((r - 3) * Math.cos(spAngle), (r - 3) * Math.sin(spAngle));
            this.ctx.stroke();
        }

        // Hub Cap
        this.ctx.fillStyle = '#A8A29E';
        this.ctx.beginPath();
        this.ctx.arc(0, 0, 3, 0, Math.PI * 2);
        this.ctx.fill();

        this.ctx.restore();
    }

    drawRobot() {
        const chassisW = 56;
        const chassisH = 22;
        const chassisY = this.floorY - 18 - chassisH;

        // 1. Draw Wheels
        this.drawWheel(this.robotX - 18, this.floorY - 10, 10, this.wheelAngle);
        this.drawWheel(this.robotX + 18, this.floorY - 10, 10, this.wheelAngle);

        // 2. Draw Chassis Body
        this.ctx.fillStyle = '#1E1B4B';
        this.ctx.strokeStyle = this.colors.purple;
        this.ctx.lineWidth = 1.5;
        
        // Round chassis corners
        this.ctx.beginPath();
        this.ctx.roundRect(this.robotX - chassisW/2, chassisY, chassisW, chassisH, 5);
        this.ctx.fill();
        this.ctx.stroke();

        // Glowing core/indicator in center
        const corePulse = 2 + Math.abs(Math.sin(Date.now() / 150)) * 2;
        this.ctx.shadowColor = this.colors.cyan;
        this.ctx.shadowBlur = 8;
        this.ctx.fillStyle = this.colors.cyan;
        this.ctx.beginPath();
        this.ctx.arc(this.robotX, chassisY + chassisH/2, corePulse, 0, Math.PI * 2);
        this.ctx.fill();
        this.ctx.shadowBlur = 0; // Reset

        // Brand details on robot
        this.ctx.font = '700 7px monospace';
        this.ctx.fillStyle = '#FFFFFF';
        this.ctx.textAlign = 'center';
        this.ctx.fillText('FEAR', this.robotX, chassisY + 8);

        // 3. Draw Arms
        const armBaseY = chassisY + 4;
        this.drawArm(this.leftArm, this.robotX - 18, armBaseY, this.colors.cyan);
        this.drawArm(this.rightArm, this.robotX + 18, armBaseY, this.colors.purple);
    }

    drawArm(arm, baseX, baseY, accentColor) {
        const joints = this.solveFK(arm, baseX, baseY);

        // Shoulder joint hub
        this.ctx.fillStyle = '#09090B';
        this.ctx.strokeStyle = accentColor;
        this.ctx.lineWidth = 1.5;
        this.ctx.beginPath();
        this.ctx.arc(baseX, baseY, 6, 0, Math.PI * 2);
        this.ctx.fill();
        this.ctx.stroke();

        // Shoulder to Elbow Segment
        this.ctx.strokeStyle = '#3F3F46';
        this.ctx.lineWidth = 7;
        this.ctx.lineCap = 'round';
        this.ctx.beginPath();
        this.ctx.moveTo(baseX, baseY);
        this.ctx.lineTo(joints.x1, joints.y1);
        this.ctx.stroke();

        this.ctx.strokeStyle = accentColor;
        this.ctx.lineWidth = 1.5;
        this.ctx.beginPath();
        this.ctx.moveTo(baseX, baseY);
        this.ctx.lineTo(joints.x1, joints.y1);
        this.ctx.stroke();

        // Elbow Joint
        this.ctx.fillStyle = '#09090B';
        this.ctx.strokeStyle = accentColor;
        this.ctx.lineWidth = 1.5;
        this.ctx.beginPath();
        this.ctx.arc(joints.x1, joints.y1, 4, 0, Math.PI * 2);
        this.ctx.fill();
        this.ctx.stroke();

        // Elbow to Wrist Segment
        this.ctx.strokeStyle = '#27272A';
        this.ctx.lineWidth = 4.5;
        this.ctx.beginPath();
        this.ctx.moveTo(joints.x1, joints.y1);
        this.ctx.lineTo(joints.x2, joints.y2);
        this.ctx.stroke();

        this.ctx.strokeStyle = '#FFFFFF';
        this.ctx.lineWidth = 1;
        this.ctx.beginPath();
        this.ctx.moveTo(joints.x1, joints.y1);
        this.ctx.lineTo(joints.x2, joints.y2);
        this.ctx.stroke();

        // Gripper (2-finger claw)
        const handAngle = arm.theta1 + arm.theta2;
        const gripSize = 8;
        const fAngle = arm.gripperOpen ? 0.38 : 0.12;

        this.ctx.strokeStyle = accentColor;
        this.ctx.lineWidth = 2;
        
        // Upper Claw
        this.ctx.beginPath();
        this.ctx.moveTo(joints.x2, joints.y2);
        this.ctx.lineTo(
            joints.x2 + gripSize * Math.cos(handAngle - fAngle),
            joints.y2 + gripSize * Math.sin(handAngle - fAngle)
        );
        this.ctx.stroke();

        // Lower Claw
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

        // Cube face
        this.ctx.fillStyle = color;
        this.ctx.shadowColor = color;
        this.ctx.shadowBlur = 6;
        this.ctx.fillRect(-this.cubeSize / 2, -this.cubeSize / 2, this.cubeSize, this.cubeSize);

        // Core highlighting borders
        this.ctx.strokeStyle = '#FFFFFF';
        this.ctx.lineWidth = 1.5;
        this.ctx.shadowBlur = 0;
        this.ctx.strokeRect(-this.cubeSize / 2, -this.cubeSize / 2, this.cubeSize, this.cubeSize);

        this.ctx.restore();
    }

    draw() {
        this.ctx.clearRect(0, 0, this.logicalWidth, this.logicalHeight);

        // 1. Draw Floor Line
        this.ctx.strokeStyle = this.colors.border;
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        this.ctx.moveTo(this.logicalWidth * 0.05, this.floorY);
        this.ctx.lineTo(this.logicalWidth * 0.95, this.floorY);
        this.ctx.stroke();

        // 2. Draw Platforms
        this.ctx.fillStyle = '#18181B';
        this.ctx.strokeStyle = this.colors.border;
        this.ctx.lineWidth = 1.5;

        // Supply Platform
        this.ctx.beginPath();
        this.ctx.roundRect(this.supplyX - 25, this.floorY, 50, 5, 2);
        this.ctx.fill();
        this.ctx.stroke();

        // Stack Platform
        this.ctx.beginPath();
        this.ctx.roundRect(this.stackX - 25, this.floorY, 50, 5, 2);
        this.ctx.fill();
        this.ctx.stroke();

        // 3. Draw dust/smoke particles
        this.ctx.fillStyle = '#3F3F46';
        this.particles.forEach((p) => {
            this.ctx.globalAlpha = p.alpha;
            this.ctx.beginPath();
            this.ctx.arc(p.x, p.y, p.size, 0, Math.PI * 2);
            this.ctx.fill();
        });
        this.ctx.globalAlpha = 1.0; // Reset

        // 4. Draw Robot
        this.drawRobot();

        // 5. Draw stacked cubes
        this.cubes.forEach((cube) => {
            let opacity = 1.0;
            if (this.state === 5 && this.stackCount >= 3) {
                opacity = Math.max(0, 1.0 - (this.timer / 45));
            }
            this.drawCube(cube.x, cube.y, cube.color, opacity);
        });

        // 6. Draw current cube being carried
        if (this.currentCube) {
            this.drawCube(this.currentCube.x, this.currentCube.y, this.currentCube.color);
        }

        // 7. Draw supply cube waiting if arm is not picking it
        if (this.state !== 1) {
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
