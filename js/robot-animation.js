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

        // Plant configurations
        this.trees = [
            { x: 65, fruitPicked: false },
            { x: 155, fruitPicked: false },
            { x: 245, fruitPicked: false }
        ];
        this.currentTreeIndex = 0;

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
        
        this.logicalWidth = rect.width || 450;
        this.logicalHeight = rect.height || 350;

        // Ground and Crate positioning
        this.floorY = this.logicalHeight * 0.85;
        this.crateX = this.logicalWidth - 55;
        this.crateY = this.floorY - 18;

        // Reset robot to starting target
        this.robotX = this.trees[0].x + 60;
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
        const chassisY = this.floorY - 32;
        const armBaseY = chassisY + 4;
        const baseLeftX = this.robotX - 16;
        const baseRightX = this.robotX + 16;

        if (ltx !== null && lty !== null) {
            const lIK = this.solveIK(ltx, lty, baseLeftX, armBaseY, 1);
            this.leftArm.targetTheta1 = lIK.theta1;
            this.leftArm.targetTheta2 = lIK.theta2;
        }
        if (rtx !== null && rty !== null) {
            const rIK = this.solveIK(rtx, rty, baseRightX, armBaseY, -1);
            this.rightArm.targetTheta1 = rIK.theta1;
            this.rightArm.targetTheta2 = rIK.theta2;
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

        const chassisY = this.floorY - 32;
        const armBaseY = chassisY + 4;
        const baseLeftX = this.robotX - 16;
        const baseRightX = this.robotX + 16;

        // Transit/Rest targets relative to arm bases
        const rTransit = { x: this.robotX + 15, y: chassisY - 40 };
        const lTransit = { x: this.robotX - 15, y: chassisY - 40 };
        const rRest = { x: this.robotX + 22, y: chassisY - 18 };
        const lRest = { x: this.robotX - 22, y: chassisY - 18 };

        const driveSpeed = 2.2;

        switch (this.state) {
            case 0: // Driving to Tree (Left/Center)
                const targetTreeX = this.trees[this.currentTreeIndex].x + 60;
                this.targetX = targetTreeX;
                const dx0 = targetTreeX - this.robotX;

                if (Math.abs(dx0) > 2) {
                    const step = Math.sign(dx0) * driveSpeed;
                    this.robotX += step;
                    this.wheelAngle += step / 10;
                    this.createDustParticles();
                } else {
                    this.robotX = targetTreeX;
                    if (this.timer > 15) {
                        this.state = 1;
                        this.timer = 0;
                    }
                }

                // Arms rest in compact positions during transit
                this.rightArm.gripperOpen = true;
                this.leftArm.gripperOpen = true;
                this.setArmTargets(lRest.x, lRest.y, rRest.x, rRest.y);
                break;

            case 1: // Pick fruit from tree branch (Right Arm reaches left)
                this.leftArm.gripperOpen = true;
                this.setArmTargets(lRest.x, lRest.y, null, null);

                // Harvest coordinates (fruit is slightly left and high)
                const targetTree = this.trees[this.currentTreeIndex];
                const fruitX = targetTree.x - 8;
                const fruitY = this.floorY - 52;

                if (this.timer < 25) {
                    this.rightArm.gripperOpen = true;
                    this.setArmTargets(null, null, fruitX, fruitY);
                } else if (this.timer < 40) {
                    // Grip fruit
                    this.rightArm.gripperOpen = false;
                    targetTree.fruitPicked = true; // Fruit vanishes from tree canopy
                    if (!this.currentCube) {
                        this.currentCube = {
                            x: fruitX,
                            y: fruitY,
                            color: this.colors.red,
                            heldBy: 'right'
                        };
                    }
                } else if (this.timer < 60) {
                    // Lift right arm
                    this.setArmTargets(null, null, rTransit.x, rTransit.y);
                } else {
                    this.state = 2;
                    this.timer = 0;
                }
                break;

            case 2: // Stationary Hand-off (at current tree location)
                const handoffX = this.robotX;
                const handoffY = chassisY - 45;

                if (this.timer < 25) {
                    this.leftArm.gripperOpen = true;
                    this.setArmTargets(handoffX, handoffY, handoffX, handoffY);
                } else if (this.timer < 40) {
                    // Left arm grips fruit
                    this.leftArm.gripperOpen = false;
                    if (this.currentCube) {
                        this.currentCube.heldBy = 'left';
                    }
                } else if (this.timer < 55) {
                    // Right arm releases
                    this.rightArm.gripperOpen = true;
                } else if (this.timer < 75) {
                    // Retract right arm, left arm keeps transit hold
                    this.setArmTargets(lTransit.x, lTransit.y, rRest.x, rRest.y);
                } else {
                    this.state = 3;
                    this.timer = 0;
                }
                break;

            case 3: // Driving to Crate (Right, logicalWidth - 115)
                const targetCrateRobotX = this.logicalWidth - 115;
                this.targetX = targetCrateRobotX;
                const dx3 = targetCrateRobotX - this.robotX;

                if (Math.abs(dx3) > 2) {
                    const step = Math.sign(dx3) * driveSpeed;
                    this.robotX += step;
                    this.wheelAngle += step / 10;
                    this.createDustParticles();

                    // Left arm holds, right arm rests
                    this.setArmTargets(lTransit.x, lTransit.y, rRest.x, rRest.y);
                } else {
                    this.robotX = targetCrateRobotX;
                    if (this.timer > 15) {
                        this.state = 4;
                        this.timer = 0;
                    }
                }
                break;

            case 4: // Drop Fruit into crate (Left arm places)
                this.rightArm.gripperOpen = true;
                this.setArmTargets(null, null, rRest.x, rRest.y);

                if (this.timer < 25) {
                    this.leftArm.gripperOpen = false;
                    this.setArmTargets(this.crateX, this.crateY, null, null);
                } else if (this.timer < 45) {
                    this.leftArm.gripperOpen = true;
                    if (this.currentCube) {
                        this.currentCube = null;
                        this.stackCount++;
                    }
                } else if (this.timer < 65) {
                    // Retract left arm to rest
                    this.setArmTargets(lRest.x, lRest.y, null, null);
                } else {
                    this.state = 5;
                    this.timer = 0;
                }
                break;

            case 5: // Reset Check / Next cycle
                this.setArmTargets(lRest.x, lRest.y, rRest.x, rRest.y);
                
                if (this.stackCount >= 3) {
                    if (this.timer > 50) {
                        // Empty crate and grow back fruits on trees
                        this.stackCount = 0;
                        this.trees.forEach(tree => tree.fruitPicked = false);
                        this.currentTreeIndex = 0;
                        this.state = 0;
                        this.timer = 0;
                    }
                } else {
                    // Select next tree to pick
                    this.currentTreeIndex = (this.currentTreeIndex + 1) % this.trees.length;
                    this.state = 0;
                    this.timer = 0;
                }
                break;
        }

        // Attach fruit coordinates to appropriate hand
        if (this.currentCube) {
            if (this.currentCube.heldBy === 'right') {
                const joints = this.solveFK(this.rightArm, baseRightX, armBaseY);
                this.currentCube.x = joints.x2;
                this.currentCube.y = joints.y2;
            } else {
                const joints = this.solveFK(this.leftArm, baseLeftX, armBaseY);
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
        this.ctx.arc(0, 0, r - 3);
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
        const chassisW = 56;
        const chassisH = 22;
        const chassisY = this.floorY - 14 - chassisH;

        // 1. Draw 4 wheels side-view rover (drawn with overlapping intervals)
        // Wheel hubs offset horizontally
        this.drawWheel(this.robotX - 22, this.floorY - 10, 10, this.wheelAngle);
        this.drawWheel(this.robotX - 8, this.floorY - 10, 10, this.wheelAngle);
        this.drawWheel(this.robotX + 8, this.floorY - 10, 10, this.wheelAngle);
        this.drawWheel(this.robotX + 22, this.floorY - 10, 10, this.wheelAngle);

        // 2. Draw Chassis Body
        this.ctx.fillStyle = '#1E1B4B';
        this.ctx.strokeStyle = this.colors.purple;
        this.ctx.lineWidth = 1.5;
        
        this.ctx.beginPath();
        this.ctx.roundRect(this.robotX - chassisW/2, chassisY, chassisW, chassisH, 5);
        this.ctx.fill();
        this.ctx.stroke();

        // Glowing core
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
        this.drawArm(this.leftArm, this.robotX - 16, armBaseY, this.colors.cyan);
        this.drawArm(this.rightArm, this.robotX + 16, armBaseY, this.colors.purple);
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
        if (this.state === 5 && this.stackCount >= 3) {
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

        // 5. Draw Robot Rover
        this.drawRobot();

        // 6. Draw current fruit carried by arm
        if (this.currentCube) {
            this.drawCube(this.currentCube.x, this.currentCube.y, this.currentCube.color);
        }
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
