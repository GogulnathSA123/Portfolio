if (typeof CanvasRenderingContext2D !== 'undefined' && !CanvasRenderingContext2D.prototype.roundRect) {
    CanvasRenderingContext2D.prototype.roundRect = function (x, y, w, h, r) {
        if (typeof r === 'undefined') r = 0;
        if (w < 2 * r) r = w / 2;
        if (h < 2 * r) r = h / 2;
        this.beginPath();
        this.moveTo(x + r, y);
        this.arcTo(x + w, y, x + w, y + h, r);
        this.arcTo(x + w, y + h, x, y + h, r);
        this.arcTo(x, y + h, x, y, r);
        this.arcTo(x, y, x + w, y, r);
        this.closePath();
        return this;
    };
}

class RobotAnimation {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        if (!this.canvas) return;
        this.ctx = this.canvas.getContext('2d');

        // Animation Settings
        this.armL1 = 44; 
        this.armL2 = 38; 
        this.cubeSize = 10; // size of the picked fruit
        
        this.colors = {
            cyan: '#00F0FF',
            purple: '#A855F7',
            red: '#EF4444',
            bg: '#0A0A0A',
            border: '#27272A',
            grid: '#1F2937'
        };

        // 2D horizontal base and state parameters
        this.robotX = 250;
        this.wheelAngle = 0;
        this.state = 0;           // 0: Driving to tree, 1: Picking fruit, 2: Hand-off, 3: Driving to crate, 4: Placing in crate, 5: Reset crate
        this.timer = 0;
        this.stackCount = 0;      // Collected fruits in crate
        this.currentCube = null;  // Fruit in transit
        this.particles = [];      // Dust particles

        // Human parameters for retrieval animation
        this.humanX = 0;
        this.humanWalkCycle = 0;
        this.humanActive = false;
        this.humanVx = 0;

        // Plant configurations
        this.trees = [
            { x: 70, fruitPicked: false },
            { x: 165, fruitPicked: false },
            { x: 260, fruitPicked: false }
        ];
        this.currentTreeIndex = 0;

        // Single central 2 DOF manipulator arm
        this.arm = {
            theta1: -Math.PI / 2,
            theta2: -Math.PI / 2,
            targetTheta1: -Math.PI / 2,
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
        
        this.logicalWidth = rect.width || 450;
        this.logicalHeight = rect.height || 350;

        // Ground and Crate positioning
        this.floorY = this.logicalHeight * 0.85;
        this.crateX = this.logicalWidth - 55;
        this.crateY = this.floorY - 18;

        // Reset robot to starting target
        this.robotX = this.trees[0].x + 55;
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

    drawRoundRect(x, y, w, h, r) {
        if (typeof r === 'undefined') r = 0;
        if (w < 2 * r) r = w / 2;
        if (h < 2 * r) r = h / 2;
        this.ctx.beginPath();
        this.ctx.moveTo(x + r, y);
        this.ctx.arcTo(x + w, y, x + w, y + h, r);
        this.ctx.arcTo(x + w, y + h, x, y + h, r);
        this.ctx.arcTo(x, y + h, x, y, r);
        this.ctx.arcTo(x, y, x + w, y, r);
        this.ctx.closePath();
    }

    setArmTarget(tx, ty) {
        const chassisY = this.floorY - 32;
        const armBaseY = chassisY + 4;
        const armBaseX = this.robotX;

        if (tx !== null && ty !== null) {
            const flip = (tx < armBaseX) ? -1 : 1;
            const ik = this.solveIK(tx, ty, armBaseX, armBaseY, flip);
            this.arm.targetTheta1 = ik.theta1;
            this.arm.targetTheta2 = ik.theta2;
        }
    }

    createDustParticles() {
        if (Math.random() < 0.4) {
            const wheelOffset = Math.random() > 0.5 ? -20 : 20;
            this.particles.push({
                x: this.robotX + wheelOffset + (Math.random() * 6 - 3),
                y: this.floorY,
                vx: -Math.sign(this.targetX - this.robotX) * (0.5 + Math.random()),
                vy: -Math.random() * 0.8,
                size: Math.random() * 3 + 1.5,
                alpha: 0.8,
                life: 0,
                maxLife: 25 + Math.random() * 15
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

        const chassisY = this.floorY - 12 - 26; // Sitting on wheels
        const armBaseY = chassisY + 6;

        // Rest and carry target for single arm relative to base
        const restX = this.robotX;
        const restY = chassisY - 15;

        const driveSpeed = 2.2;

        switch (this.state) {
            case 0: // Driving to Tree (Left/Center)
                const targetTreeX = this.trees[this.currentTreeIndex].x + 55;
                this.targetX = targetTreeX;
                const dx0 = targetTreeX - this.robotX;

                if (Math.abs(dx0) > 1.5) {
                    const speed = Math.max(0.5, Math.min(driveSpeed, Math.abs(dx0) * 0.15));
                    const step = Math.sign(dx0) * speed;
                    this.robotX += step;
                    this.wheelAngle += step / 12;
                    this.createDustParticles();
                } else {
                    this.robotX = targetTreeX;
                    this.state = 1; // Arrived -> Scan fruit
                    this.timer = 0;
                }

                // Arm rests in folded position during transit
                this.arm.gripperOpen = true;
                this.setArmTarget(restX, restY);
                break;

            case 1: // Scanning Target Fruit (Pause for planning)
                this.arm.gripperOpen = true;
                this.setArmTarget(restX, restY);
                if (this.timer > 30) {
                    this.state = 2; // Begin grasp reach
                    this.timer = 0;
                }
                break;

            case 2: // Reaching & Plucking Fruit
                const targetTree = this.trees[this.currentTreeIndex];
                const fruitX = targetTree.x - 8;
                const fruitY = this.floorY - 52;

                if (this.timer < 30) {
                    this.arm.gripperOpen = true;
                    this.setArmTarget(fruitX, fruitY);
                } else if (this.timer < 50) {
                    // Close gripper
                    this.arm.gripperOpen = false;
                    targetTree.fruitPicked = true; // Fruit vanishes
                    if (!this.currentCube) {
                        this.currentCube = {
                            x: fruitX,
                            y: fruitY,
                            color: this.colors.red
                        };
                    }
                    this.setArmTarget(fruitX, fruitY);
                } else {
                    this.state = 3;
                    this.timer = 0;
                }
                break;

            case 3: // Retract arm to carry position
                this.arm.gripperOpen = false;
                this.setArmTarget(restX, restY);
                if (this.timer > 25) {
                    this.state = 4; // Start transit to crate
                    this.timer = 0;
                }
                break;

            case 4: // Driving to Crate (Right side)
                const targetCrateRobotX = this.logicalWidth - 110;
                this.targetX = targetCrateRobotX;
                const dx4 = targetCrateRobotX - this.robotX;

                if (Math.abs(dx4) > 1.5) {
                    const speed = Math.max(0.5, Math.min(driveSpeed, Math.abs(dx4) * 0.15));
                    const step = Math.sign(dx4) * speed;
                    this.robotX += step;
                    this.wheelAngle += step / 12;
                    this.createDustParticles();
                    
                    // Arm carries fruit in carry position
                    this.arm.gripperOpen = false;
                    this.setArmTarget(restX, restY);
                } else {
                    this.robotX = targetCrateRobotX;
                    this.state = 5; // Arrived -> Scan Crate
                    this.timer = 0;
                }
                break;

            case 5: // Scanning Crate (Pause for planning)
                this.arm.gripperOpen = false;
                this.setArmTarget(restX, restY);
                if (this.timer > 30) {
                    this.state = 6; // Begin drop reach
                    this.timer = 0;
                }
                break;

            case 6: // Drop Fruit into crate
                if (this.timer < 30) {
                    this.arm.gripperOpen = false;
                    this.setArmTarget(this.crateX, this.crateY);
                } else if (this.timer < 55) {
                    this.arm.gripperOpen = true;
                    if (this.currentCube) {
                        this.currentCube = null;
                        this.stackCount++;
                    }
                    this.setArmTarget(this.crateX, this.crateY);
                } else if (this.timer < 85) {
                    // Retract arm back to rest
                    this.setArmTarget(restX, restY);
                } else {
                    this.state = 7;
                    this.timer = 0;
                }
                break;

            case 7: // Reset Check / Next cycle
                this.arm.gripperOpen = true;
                this.setArmTarget(restX, restY);
                
                if (this.stackCount >= 3) {
                    // Crate is full -> Transition to Human Retrieval
                    this.humanX = this.logicalWidth + 30;
                    this.humanWalkCycle = 0;
                    this.humanActive = true;
                    this.humanVx = -1.2;
                    this.state = 8;
                    this.timer = 0;
                } else {
                    this.currentTreeIndex = (this.currentTreeIndex + 1) % this.trees.length;
                    this.state = 0;
                    this.timer = 0;
                }
                break;

            case 8: // Human walking in to retrieve crate
                this.arm.gripperOpen = true;
                this.setArmTarget(restX, restY);

                this.humanVx = -1.2;
                this.humanX += this.humanVx;
                this.humanWalkCycle += 0.12;

                if (this.humanX <= this.crateX) {
                    this.humanX = this.crateX;
                    this.state = 9;
                    this.timer = 0;
                }
                break;

            case 9: // Human picking up crate (0.5s pause to lift)
                this.arm.gripperOpen = true;
                this.setArmTarget(restX, restY);

                if (this.timer > 30) {
                    this.state = 10;
                    this.timer = 0;
                }
                break;

            case 10: // Human walking out carrying crate
                this.arm.gripperOpen = true;
                this.setArmTarget(restX, restY);

                this.humanVx = 1.2;
                this.humanX += this.humanVx;
                this.humanWalkCycle += 0.12;

                // Crate tracks the human
                this.crateX = this.humanX;
                this.crateY = this.floorY - 24; // Lifted up to hand height

                if (this.humanX >= this.logicalWidth + 30) {
                    // Reset environment and loop
                    this.humanActive = false;
                    this.stackCount = 0;
                    this.trees.forEach(tree => tree.fruitPicked = false);
                    this.currentTreeIndex = 0;
                    this.crateX = this.logicalWidth - 55;
                    this.crateY = this.floorY - 18;
                    this.state = 0;
                    this.timer = 0;
                }
                break;
        }

        // Attach fruit coordinates to arm hand
        if (this.currentCube) {
            const joints = this.solveFK(this.arm, this.robotX, armBaseY);
            this.currentCube.x = joints.x2;
            this.currentCube.y = joints.y2;
        }

        // Smoothly interpolate angles
        const lerpSpeed = 0.15;
        this.arm.theta1 += (this.arm.targetTheta1 - this.arm.theta1) * lerpSpeed;
        this.arm.theta2 += (this.arm.targetTheta2 - this.arm.theta2) * lerpSpeed;
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

        // Glowing cyan spokes
        this.ctx.strokeStyle = this.colors.cyan;
        this.ctx.lineWidth = 1.0;
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
        const chassisW = 44;
        const chassisH = 26;
        const chassisY = this.floorY - 12 - chassisH;

        // 1. Draw 2 wheels (differential-drive balancing robot design)
        this.drawWheel(this.robotX - 18, this.floorY - 12, 12, this.wheelAngle);
        this.drawWheel(this.robotX + 18, this.floorY - 12, 12, this.wheelAngle);

        // 2. Draw Chassis Body
        this.ctx.fillStyle = '#1E1B4B';
        this.ctx.strokeStyle = this.colors.purple;
        this.ctx.lineWidth = 1.5;
        
        this.drawRoundRect(this.robotX - chassisW/2, chassisY, chassisW, chassisH, 6);
        this.ctx.fill();
        this.ctx.stroke();

        // Glowing core
        const corePulse = 3 + Math.abs(Math.sin(Date.now() / 150)) * 2;
        this.ctx.shadowColor = this.colors.cyan;
        this.ctx.shadowBlur = 8;
        this.ctx.fillStyle = this.colors.cyan;
        this.ctx.beginPath();
        this.ctx.arc(this.robotX, chassisY + chassisH/2 - 2, corePulse, 0, Math.PI * 2);
        this.ctx.fill();
        this.ctx.shadowBlur = 0; // Reset

        // Brand details on robot
        this.ctx.font = '700 7px monospace';
        this.ctx.fillStyle = '#FFFFFF';
        this.ctx.textAlign = 'center';
        this.ctx.fillText('FEAR', this.robotX, chassisY + 8);
        
        // Small camera sensor mount (VLA eye) on top
        this.ctx.fillStyle = '#09090B';
        this.ctx.strokeStyle = this.colors.cyan;
        this.ctx.lineWidth = 1.0;
        this.ctx.beginPath();
        this.ctx.rect(this.robotX - 6, chassisY - 6, 12, 6);
        this.ctx.fill();
        this.ctx.stroke();

        this.ctx.fillStyle = this.colors.cyan;
        this.ctx.beginPath();
        this.ctx.arc(this.robotX, chassisY - 3, 2, 0, Math.PI * 2);
        this.ctx.fill();

        // 3. Draw Single Arm
        const armBaseY = chassisY + 6;
        this.drawArm(this.arm, this.robotX, armBaseY, this.colors.cyan);
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

        // Gripper claws
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

    drawCube(x, y, color, opacity = 1.0, scale = 1.0) {
        this.ctx.save();
        this.ctx.translate(x, y);
        this.ctx.globalAlpha = opacity;

        const size = this.cubeSize * scale;

        // Red round fruit shape
        this.ctx.fillStyle = color;
        this.ctx.shadowColor = color;
        this.ctx.shadowBlur = 6;
        this.ctx.beginPath();
        this.ctx.arc(0, 0, size/2, 0, Math.PI * 2);
        this.ctx.fill();

        // Fruit outline highlight
        this.ctx.strokeStyle = '#FFFFFF';
        this.ctx.lineWidth = 1.0;
        this.ctx.shadowBlur = 0;
        this.ctx.stroke();

        // Small green leaf
        this.ctx.strokeStyle = '#15803D';
        this.ctx.lineWidth = 1.2;
        this.ctx.beginPath();
        this.ctx.moveTo(0, -size/2);
        this.ctx.quadraticCurveTo(-2, -size/2 - 3, -4, -size/2 - 1);
        this.ctx.stroke();

        this.ctx.restore();
    }

    drawCrate(cx, cy) {
        const w = 34;
        const h = 18;
        
        // Wood slats bin
        this.ctx.fillStyle = '#78350F';
        this.ctx.strokeStyle = '#D97706';
        this.ctx.lineWidth = 1.5;

        this.ctx.beginPath();
        this.ctx.rect(cx - w/2, cy, w, h);
        this.ctx.fill();
        this.ctx.stroke();

        // Horizontal wood boards division
        this.ctx.beginPath();
        this.ctx.moveTo(cx - w/2, cy + h * 0.35);
        this.ctx.lineTo(cx + w/2, cy + h * 0.35);
        this.ctx.moveTo(cx - w/2, cy + h * 0.7);
        this.ctx.lineTo(cx + w/2, cy + h * 0.7);
        this.ctx.strokeStyle = '#451A03';
        this.ctx.lineWidth = 1.0;
        this.ctx.stroke();

        // Label
        this.ctx.fillStyle = 'rgba(255, 255, 255, 0.4)';
        this.ctx.font = '600 5px sans-serif';
        this.ctx.textAlign = 'center';
        this.ctx.fillText('CROP', cx, cy + h - 3);
    }

    drawFruitsInCrate(cx, cy, count) {
        const fr = 4.0;
        
        // Crate fruit coordinates
        const positions = [
            { dx: -7, dy: 13 },
            { dx: 6, dy: 13 },
            { dx: -1, dy: 6 }
        ];

        for (let i = 0; i < count; i++) {
            if (i >= positions.length) break;
            const px = cx + positions[i].dx;
            const py = cy + positions[i].dy;
            
            this.ctx.save();
            this.ctx.translate(px, py);
            this.ctx.fillStyle = this.colors.red;
            this.ctx.strokeStyle = '#FFFFFF';
            this.ctx.lineWidth = 0.8;
            this.ctx.beginPath();
            this.ctx.arc(0, 0, fr, 0, Math.PI * 2);
            this.ctx.fill();
            this.ctx.stroke();
            this.ctx.restore();
        }
    }

    drawHuman() {
        const hipX = this.humanX;
        const hipY = this.floorY - 16;
        const headY = this.floorY - 42;
        const shoulderY = this.floorY - 34;

        // Walk cycle leg/arm swings
        const legSwing = Math.sin(this.humanWalkCycle) * 0.35;
        const armSwing = Math.cos(this.humanWalkCycle) * 0.35;
        const dir = Math.sign(this.humanVx || -1);

        // 1. Draw Head
        this.ctx.fillStyle = '#E4E4E7';
        this.ctx.beginPath();
        this.ctx.arc(this.humanX, headY, 5, 0, Math.PI * 2);
        this.ctx.fill();

        // Cybernetic visor (cyan)
        this.ctx.fillStyle = this.colors.cyan;
        this.ctx.beginPath();
        this.ctx.arc(this.humanX + dir * 3, headY - 1, 1.5, 0, Math.PI * 2);
        this.ctx.fill();

        // 2. Draw Torso
        this.ctx.strokeStyle = '#A1A1AA';
        this.ctx.lineWidth = 3;
        this.ctx.lineCap = 'round';
        this.ctx.beginPath();
        this.ctx.moveTo(this.humanX, shoulderY);
        this.ctx.lineTo(hipX, hipY);
        this.ctx.stroke();

        // 3. Draw Legs
        const legLength = 16;
        
        // Left Leg
        const lAngle = (this.state === 8 || this.state === 10) ? legSwing : 0;
        const lFootX = hipX + Math.sin(lAngle) * legLength;
        const lFootY = hipY + Math.cos(lAngle) * legLength;
        this.ctx.strokeStyle = '#A1A1AA';
        this.ctx.lineWidth = 3;
        this.ctx.beginPath();
        this.ctx.moveTo(hipX, hipY);
        this.ctx.lineTo(lFootX, lFootY);
        this.ctx.stroke();

        // Right Leg
        const rAngle = (this.state === 8 || this.state === 10) ? -legSwing : 0;
        const rFootX = hipX + Math.sin(rAngle) * legLength;
        const rFootY = hipY + Math.cos(rAngle) * legLength;
        this.ctx.beginPath();
        this.ctx.moveTo(hipX, hipY);
        this.ctx.lineTo(rFootX, rFootY);
        this.ctx.stroke();

        // 4. Draw Arms
        const armLength = 12;
        this.ctx.strokeStyle = '#D4D4D8';
        this.ctx.lineWidth = 2.5;

        if (this.state === 9 || this.state === 10) {
            // Reaching out to hold crate
            const handX = this.humanX + dir * 8;
            const handY = this.floorY - 24; // level with carrying crate height

            this.ctx.beginPath();
            this.ctx.moveTo(this.humanX, shoulderY);
            this.ctx.lineTo(handX, handY);
            this.ctx.stroke();
        } else {
            // Walking arm swing
            const lArmAngle = armSwing;
            const lHandX = this.humanX + Math.sin(lArmAngle) * armLength;
            const lHandY = shoulderY + Math.cos(lArmAngle) * armLength;
            this.ctx.beginPath();
            this.ctx.moveTo(this.humanX, shoulderY);
            this.ctx.lineTo(lHandX, lHandY);
            this.ctx.stroke();

            const rArmAngle = -armSwing;
            const rHandX = this.humanX + Math.sin(rArmAngle) * armLength;
            const rHandY = shoulderY + Math.cos(rArmAngle) * armLength;
            this.ctx.beginPath();
            this.ctx.moveTo(this.humanX, shoulderY);
            this.ctx.lineTo(rHandX, rHandY);
            this.ctx.stroke();
        }
    }

    drawTree(treeX) {
        const py = this.floorY;

        // Tree brown trunk
        this.ctx.strokeStyle = '#451A03';
        this.ctx.lineWidth = 4;
        this.ctx.beginPath();
        this.ctx.moveTo(treeX, py);
        this.ctx.lineTo(treeX, py - 40);
        this.ctx.stroke();

        // Green leaf canopy circles
        this.ctx.fillStyle = '#15803D';
        this.ctx.beginPath();
        this.ctx.arc(treeX, py - 40, 16, 0, Math.PI * 2);
        this.ctx.arc(treeX - 10, py - 48, 12, 0, Math.PI * 2);
        this.ctx.arc(treeX + 10, py - 48, 12, 0, Math.PI * 2);
        this.ctx.fill();

        // Canopy shadow overlay
        this.ctx.fillStyle = '#166534';
        this.ctx.beginPath();
        this.ctx.arc(treeX - 5, py - 35, 9, 0, Math.PI * 2);
        this.ctx.arc(treeX + 5, py - 35, 9, 0, Math.PI * 2);
        this.ctx.fill();

        // 2 static background fruits
        this.ctx.fillStyle = this.colors.red;
        this.ctx.strokeStyle = '#FFFFFF';
        this.ctx.lineWidth = 0.6;
        
        // Static Fruit 1
        this.ctx.beginPath();
        this.ctx.arc(treeX + 6, py - 38, 3, 0, Math.PI * 2);
        this.ctx.fill();
        this.ctx.stroke();

        // Static Fruit 2
        this.ctx.beginPath();
        this.ctx.arc(treeX - 6, py - 30, 3, 0, Math.PI * 2);
        this.ctx.fill();
        this.ctx.stroke();
    }

    drawHUD() {
        let stateLabel = '';
        let queryText = '';
        let telemetryAction = '';

        switch (this.state) {
            case 0:
                stateLabel = 'NAV_TO_PICK';
                queryText = `"Locate tree ${this.currentTreeIndex + 1} and navigate to harvest coordinate."`;
                telemetryAction = Math.abs(this.targetX - this.robotX) > 2 ? 'DRIVING (2.2m/s)' : 'ALIGNING';
                break;
            case 1:
                stateLabel = 'SCAN_OBJECT';
                queryText = `"Run spatial scanning pipeline on targeted apple cluster."`;
                telemetryAction = 'VLA INFERENCE...';
                break;
            case 2:
                stateLabel = 'GRASP_OBJECT';
                queryText = `"Move manipulator to target apple coordinate and grasp it."`;
                telemetryAction = 'EXTENDING';
                break;
            case 3:
                stateLabel = 'RETRACT_ARM';
                queryText = `"Secure crop payload and retract arm to transit configuration."`;
                telemetryAction = 'RETRACTING';
                break;
            case 4:
                stateLabel = 'NAV_TO_CRATE';
                queryText = `"Navigate to crop crate and align deposit vectors."`;
                telemetryAction = Math.abs(this.targetX - this.robotX) > 2 ? 'DRIVING (2.2m/s)' : 'ALIGNING';
                break;
            case 5:
                stateLabel = 'SCAN_CRATE';
                queryText = `"Scan collection box coordinates for collision-free deposit path."`;
                telemetryAction = 'VLA INFERENCE...';
                break;
            case 6:
                stateLabel = 'DEPOSIT_OBJECT';
                queryText = `"Extend manipulator and release apple at crate coordinate."`;
                telemetryAction = 'DEPOSITING';
                break;
            case 7:
                stateLabel = 'RESET_CHECK';
                queryText = `"Evaluating crop collection payload status and capacities."`;
                telemetryAction = 'COMPUTING';
                break;
            case 8:
                stateLabel = 'WAIT_FOR_HUMAN';
                queryText = `"Container capacity reached. Dispatched human operator to retrieve crop crate."`;
                telemetryAction = 'STANDBY';
                break;
            case 9:
                stateLabel = 'HUMAN_CONTACT';
                queryText = `"Operator detected in workspace. Actuators locked. Suspending all autonomous drive."`;
                telemetryAction = 'LOCKDOWN';
                break;
            case 10:
                stateLabel = 'CROP_EXTRACTED';
                queryText = `"Crop crate successfully extracted. Awaiting operator exit and environment reset."`;
                telemetryAction = 'STANDBY';
                break;
        }

        // Draw top instruction box
        this.ctx.save();
        this.ctx.fillStyle = 'rgba(10, 10, 12, 0.75)';
        this.ctx.strokeStyle = 'rgba(0, 240, 255, 0.25)';
        this.ctx.lineWidth = 1.5;
        this.drawRoundRect(10, 10, this.logicalWidth - 20, 32, 5);
        this.ctx.fill();
        this.ctx.stroke();

        // Instruction Header
        this.ctx.font = '700 7px monospace';
        this.ctx.fillStyle = this.colors.cyan;
        this.ctx.textAlign = 'left';
        this.ctx.fillText('EMBODIED VLA MODEL COMMAND INPUT', 18, 22);

        // Instruction Query Text
        this.ctx.font = '500 8.5px monospace';
        this.ctx.fillStyle = '#FFFFFF';
        this.ctx.fillText(queryText, 18, 34);

        // Draw telemetry panel
        this.ctx.fillStyle = 'rgba(10, 10, 12, 0.75)';
        this.ctx.strokeStyle = 'rgba(0, 240, 255, 0.2)';
        this.drawRoundRect(10, 50, 140, 70, 5);
        this.ctx.fill();
        this.ctx.stroke();

        // Telemetry Text lines
        this.ctx.font = '700 7px monospace';
        this.ctx.fillStyle = 'rgba(255, 255, 255, 0.4)';
        this.ctx.fillText('SYSTEM TELEMETRY', 18, 62);

        this.ctx.fillStyle = '#FFFFFF';
        this.ctx.font = '600 7px monospace';
        this.ctx.fillText(`MODEL : ACT-MAMBA VLA`, 18, 72);
        
        this.ctx.fillStyle = this.colors.cyan;
        this.ctx.fillText(`STATE : ${stateLabel}`, 18, 82);
        
        this.ctx.fillStyle = '#EAB308'; // yellow
        this.ctx.fillText(`ACTION: ${telemetryAction}`, 18, 92);

        this.ctx.fillStyle = '#FFFFFF';
        this.ctx.fillText(`THETA : ${(this.arm.theta1 * 180 / Math.PI).toFixed(0)}°, ${(this.arm.theta2 * 180 / Math.PI).toFixed(0)}°`, 18, 102);

        this.ctx.fillStyle = this.arm.gripperOpen ? '#22C55E' : '#EF4444';
        this.ctx.fillText(`GRIP  : ${this.arm.gripperOpen ? 'OPEN' : 'CLOSED'}`, 18, 112);

        this.ctx.restore();
    }

    draw() {
        this.ctx.clearRect(0, 0, this.logicalWidth, this.logicalHeight);

        // 1. Draw floor soil boundary
        const floorGrad = this.ctx.createLinearGradient(0, this.floorY, 0, this.logicalHeight);
        floorGrad.addColorStop(0, '#060B06');
        floorGrad.addColorStop(1, '#0F2010');
        this.ctx.fillStyle = floorGrad;
        this.ctx.fillRect(0, this.floorY, this.logicalWidth, this.logicalHeight - this.floorY);

        this.ctx.strokeStyle = '#142918';
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        this.ctx.moveTo(this.logicalWidth * 0.05, this.floorY);
        this.ctx.lineTo(this.logicalWidth * 0.95, this.floorY);
        this.ctx.stroke();

        // 2. Draw Orchard Trees & Fruits
        this.trees.forEach((tree, idx) => {
            this.drawTree(tree.x);
            // Draw harvest target fruit if not plucked
            if (!tree.fruitPicked) {
                this.drawCube(tree.x - 8, this.floorY - 52, this.colors.red);
            }
        });

        // 3. Draw Crate & Placed fruits
        let crateOpacity = 1.0;
        if (this.state === 7 && this.stackCount >= 3) {
            crateOpacity = Math.max(0.3, 1.0 - (this.timer / 50));
        }
        this.ctx.globalAlpha = crateOpacity;
        this.drawCrate(this.crateX, this.crateY);
        this.drawFruitsInCrate(this.crateX, this.crateY, this.stackCount);
        this.ctx.globalAlpha = 1.0; // Reset

        // 4. Draw dust particles
        this.ctx.fillStyle = '#3F6212';
        this.particles.forEach((p) => {
            this.ctx.globalAlpha = p.alpha;
            this.ctx.beginPath();
            this.ctx.arc(p.x, p.y, p.size, 0, Math.PI * 2);
            this.ctx.fill();
        });
        this.ctx.globalAlpha = 1.0; // Reset

        // 5. Draw Vision Target Bounding Box
        if (this.state === 0 || this.state === 1 || this.state === 2) {
            const targetTree = this.trees[this.currentTreeIndex];
            const fx = targetTree.x - 8;
            const fy = this.floorY - 52;
            
            this.ctx.save();
            this.ctx.strokeStyle = '#22C55E';
            this.ctx.lineWidth = 1;
            this.ctx.setLineDash([2, 2]);
            this.ctx.beginPath();
            this.ctx.rect(fx - 8, fy - 8, 16, 16);
            this.ctx.stroke();
            
            this.ctx.fillStyle = '#22C55E';
            this.ctx.font = '600 7px monospace';
            this.ctx.fillText('[Apple: 0.99]', fx - 8, fy - 11);
            this.ctx.restore();
        } else if (this.state === 4 || this.state === 5 || this.state === 6) {
            const cx = this.crateX;
            const cy = this.crateY;
            
            this.ctx.save();
            this.ctx.strokeStyle = '#3B82F6';
            this.ctx.lineWidth = 1;
            this.ctx.setLineDash([2, 2]);
            this.ctx.beginPath();
            this.ctx.rect(cx - 20, cy - 2, 40, 22);
            this.ctx.stroke();
            
            this.ctx.fillStyle = '#3B82F6';
            this.ctx.font = '600 7px monospace';
            this.ctx.fillText('[Crate: 0.98]', cx - 20, cy - 5);
            this.ctx.restore();
        }

        // 6. Draw Camera Scanning Beam (Sensory Input)
        const chassisH = 26;
        const chassisY = this.floorY - 12 - chassisH;
        const cameraX = this.robotX;
        const cameraY = chassisY - 2;

        if (this.state === 1) {
            const targetTree = this.trees[this.currentTreeIndex];
            const fx = targetTree.x - 8;
            const fy = this.floorY - 52;
            
            this.ctx.save();
            const scanGrad = this.ctx.createRadialGradient(cameraX, cameraY, 2, fx, fy, 40);
            scanGrad.addColorStop(0, 'rgba(0, 240, 255, 0.4)');
            scanGrad.addColorStop(1, 'rgba(0, 240, 255, 0.02)');
            this.ctx.fillStyle = scanGrad;
            
            this.ctx.beginPath();
            this.ctx.moveTo(cameraX, cameraY);
            this.ctx.lineTo(fx - 12, fy - 12);
            this.ctx.lineTo(fx + 12, fy + 12);
            this.ctx.closePath();
            this.ctx.fill();
            this.ctx.restore();
        } else if (this.state === 5) {
            const fx = this.crateX;
            const fy = this.crateY;
            
            this.ctx.save();
            const scanGrad = this.ctx.createRadialGradient(cameraX, cameraY, 2, fx, fy, 40);
            scanGrad.addColorStop(0, 'rgba(0, 240, 255, 0.4)');
            scanGrad.addColorStop(1, 'rgba(0, 240, 255, 0.02)');
            this.ctx.fillStyle = scanGrad;
            
            this.ctx.beginPath();
            this.ctx.moveTo(cameraX, cameraY);
            this.ctx.lineTo(fx - 20, fy - 5);
            this.ctx.lineTo(fx + 20, fy - 5);
            this.ctx.closePath();
            this.ctx.fill();
            this.ctx.restore();
        }

        // 7. Draw Robot Rover
        this.drawRobot();

        // 8. Draw current fruit carried by arm
        if (this.currentCube) {
            this.drawCube(this.currentCube.x, this.currentCube.y, this.currentCube.color);
        }

        // Draw Human Operator if active
        if (this.humanActive) {
            this.drawHuman();
        }

        // 9. Draw VLA model interface HUD
        this.drawHUD();
    }

    loop() {
        this.update();
        this.draw();
        requestAnimationFrame(() => this.loop());
    }
}

// Safe instantiation on load
const initRobotAnimation = () => {
    try {
        console.log("Farming Robot Animation Initializing...");
        new RobotAnimation('robotCanvas');
    } catch (e) {
        console.error("Failed to initialize RobotAnimation:", e);
    }
};

if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initRobotAnimation);
} else {
    initRobotAnimation();
}
