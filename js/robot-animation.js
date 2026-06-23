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

        // Kinematics and visual settings (at base scale 1.0)
        this.armL1 = 44; 
        this.armL2 = 38; 
        this.cubeSize = 10; // size of the picked fruit
        
        this.colors = {
            cyan: '#00F0FF',
            purple: '#A855F7',
            red: '#EF4444',     // Red fruit color
            bg: '#0A0A0A',
            border: '#142918',  // Dark green border
            grid: '#1F2937'
        };

        // Perspective settings
        this.z = 0.8;             // Depth coordinate (ranges from 0.28 to 0.82)
        this.wheelAngle = 0;
        this.state = 0;           // 0: Drive backward, 1: Pick fruit, 2: Drive center & hand-off, 3: Drive forward, 4: Place fruit in box, 5: Reset crate
        this.timer = 0;
        this.stackCount = 0;      // Collected fruits count
        this.currentCube = null;  // Fruit in transit
        this.particles = [];      // Dust particles

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

        // Vanishing point coordinates for warehouse perspective
        this.vpX = this.logicalWidth * 0.5;
        this.vpY = this.logicalHeight * 0.32;

        // Pickup / Drop stations (absolute coordinates in perspective)
        this.zPick = 0.28;
        this.zDrop = 0.82;

        const floorY = (z) => this.vpY + (this.logicalHeight * 0.95 - this.vpY) * z;
        
        // Supply fruit hangs on Left Canopy branch in background
        this.supplyX = this.vpX - 35 * this.zPick;
        this.supplyY = floorY(this.zPick) - 34 * this.zPick;

        // Collection bin sits on Right Aisle floor in foreground
        this.boxX = this.vpX + 45 * this.zDrop;
        this.boxY = floorY(this.zDrop) - 10 * this.zDrop;
    }

    // Solve Inverse Kinematics for a 2-segment arm (normalized to unscaled space)
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

    // Forward Kinematics (returns absolute screen coordinates for joints)
    solveFK(arm, baseX, baseY, scale = 1.0) {
        const l1 = this.armL1 * scale;
        const l2 = this.armL2 * scale;
        const x1 = baseX + l1 * Math.cos(arm.theta1);
        const y1 = baseY + l1 * Math.sin(arm.theta1);
        const x2 = x1 + l2 * Math.cos(arm.theta1 + arm.theta2);
        const y2 = y1 + l2 * Math.sin(arm.theta1 + arm.theta2);
        return { x1, y1, x2, y2 };
    }

    // Convenience function to solve scaled IK targets
    setArmTargetsNormalized(ltx_rel, lty_rel, rtx_rel, rty_rel) {
        if (ltx_rel !== null && lty_rel !== null) {
            const lIK = this.solveIK(ltx_rel, lty_rel, 0, 0, 1);
            this.leftArm.targetTheta1 = lIK.theta1;
            this.leftArm.targetTheta2 = lIK.theta2;
        }
        if (rtx_rel !== null && rty_rel !== null) {
            const rIK = this.solveIK(rtx_rel, rty_rel, 0, 0, -1);
            this.rightArm.targetTheta1 = rIK.theta1;
            this.rightArm.targetTheta2 = rIK.theta2;
        }
    }

    createDustParticles() {
        if (Math.random() < 0.4) {
            const scale = this.z;
            const floorY = this.vpY + (this.logicalHeight * 0.95 - this.vpY) * this.z;
            const wheelOffset = Math.random() > 0.5 ? -20 : 20;
            this.particles.push({
                x: this.vpX + wheelOffset * scale + (Math.random() * 6 - 3) * scale,
                y: floorY,
                vx: (Math.random() * 1.5 - 0.75) * scale,
                vy: -Math.random() * 0.8 * scale,
                size: (Math.random() * 3 + 1.5) * scale,
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

        const scale = this.z;
        const floorY = this.vpY + (this.logicalHeight * 0.95 - this.vpY) * this.z;
        const chassisY = floorY - 14 * scale;
        const chassisTopY = chassisY - 18 * scale;

        // Base coordinate for left/right arms in screen space
        const baseLeftX = this.vpX - 16 * scale;
        const baseLeftY = chassisTopY + 3 * scale;
        const baseRightX = this.vpX + 16 * scale;
        const baseRightY = chassisTopY + 3 * scale;

        // Normalized relative targets (for unscaled IK solver space)
        const rTransit = { x: 15, y: -40 };
        const lTransit = { x: -15, y: -40 };
        const rRest = { x: 22, y: -20 };
        const lRest = { x: -22, y: -20 };

        const driveSpeedZ = 0.007;

        switch (this.state) {
            case 0: // Driving Inwards / Backward to Pickup (z decreases)
                if (this.z > this.zPick) {
                    this.z -= driveSpeedZ;
                    this.wheelAngle -= 0.06;
                    this.createDustParticles();
                } else {
                    this.z = this.zPick;
                    if (this.timer > 15) {
                        this.state = 1;
                        this.timer = 0;
                    }
                }

                // Arms remain in compact transit pose
                this.rightArm.gripperOpen = true;
                this.leftArm.gripperOpen = true;
                this.setArmTargetsNormalized(lRest.x, lRest.y, rRest.x, rRest.y);
                break;

            case 1: // Pick Fruit from branch canopy (Right arm reaches)
                this.leftArm.gripperOpen = true;
                this.setArmTargetsNormalized(lRest.x, lRest.y, null, null);

                // Target coordinates normalized to unscaled space
                const rxRelPick = (this.supplyX - baseRightX) / scale;
                const ryRelPick = (this.supplyY - baseRightY) / scale;

                if (this.timer < 25) {
                    this.rightArm.gripperOpen = true;
                    this.setArmTargetsNormalized(null, null, rxRelPick, ryRelPick);
                } else if (this.timer < 40) {
                    this.rightArm.gripperOpen = false;
                    if (!this.currentCube) {
                        this.currentCube = {
                            x: this.supplyX,
                            y: this.supplyY,
                            color: this.colors.red,
                            heldBy: 'right'
                        };
                    }
                } else if (this.timer < 60) {
                    // Lift right arm with fruit
                    this.setArmTargetsNormalized(null, null, rTransit.x, rTransit.y);
                } else {
                    this.state = 2;
                    this.timer = 0;
                }
                break;

            case 2: // Drive forward to Center (z = 0.53) & Hand-off
                const zCenter = 0.53;
                if (this.z < zCenter) {
                    this.z += driveSpeedZ;
                    this.wheelAngle += 0.06;
                    this.createDustParticles();
                    
                    // Maintain transit hold
                    this.setArmTargetsNormalized(lRest.x, lRest.y, rTransit.x, rTransit.y);
                } else {
                    this.z = zCenter;
                    
                    // Stationary Hand-off relative positions
                    const lHandoff = { x: 16, y: -45 };
                    const rHandoff = { x: -16, y: -45 };

                    if (this.timer < 25) {
                        this.leftArm.gripperOpen = true;
                        this.setArmTargetsNormalized(lHandoff.x, lHandoff.y, rHandoff.x, rHandoff.y);
                    } else if (this.timer < 40) {
                        // Left arm grips
                        this.leftArm.gripperOpen = false;
                        if (this.currentCube) {
                            this.currentCube.heldBy = 'left';
                        }
                    } else if (this.timer < 55) {
                        // Right arm releases
                        this.rightArm.gripperOpen = true;
                    } else if (this.timer < 75) {
                        // Retract right, keep left transit hold
                        this.setArmTargetsNormalized(lTransit.x, lTransit.y, rRest.x, rRest.y);
                    } else {
                        this.state = 3;
                        this.timer = 0;
                    }
                }
                break;

            case 3: // Drive forward to Drop Zone (Foreground, z = 0.82)
                if (this.z < this.zDrop) {
                    this.z += driveSpeedZ;
                    this.wheelAngle += 0.06;
                    this.createDustParticles();

                    // Left arm holds fruit, right arm rests
                    this.setArmTargetsNormalized(lTransit.x, lTransit.y, rRest.x, rRest.y);
                } else {
                    this.z = this.zDrop;
                    if (this.timer > 15) {
                        this.state = 4;
                        this.timer = 0;
                    }
                }
                break;

            case 4: // Drop Fruit in box/crate (Left arm places)
                this.rightArm.gripperOpen = true;
                this.setArmTargetsNormalized(null, null, rRest.x, rRest.y);

                // Place fruit over the crate
                const lxRelPlace = (this.boxX - baseLeftX) / scale;
                const lyRelPlace = (this.boxY - 4 * scale - baseLeftY) / scale;

                if (this.timer < 25) {
                    this.leftArm.gripperOpen = false;
                    this.setArmTargetsNormalized(lxRelPlace, lyRelPlace, null, null);
                } else if (this.timer < 45) {
                    // Release fruit
                    this.leftArm.gripperOpen = true;
                    if (this.currentCube) {
                        this.currentCube = null;
                        this.stackCount++;
                    }
                } else if (this.timer < 65) {
                    // Retract left arm to rest
                    this.setArmTargetsNormalized(lRest.x, lRest.y, null, null);
                } else {
                    this.state = 5;
                    this.timer = 0;
                }
                break;

            case 5: // Reset bin / Wait
                this.setArmTargetsNormalized(lRest.x, lRest.y, rRest.x, rRest.y);
                if (this.stackCount >= 3) {
                    if (this.timer > 50) {
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

        // Attach current cube to correct gripper in screen coordinates
        if (this.currentCube) {
            if (this.currentCube.heldBy === 'right') {
                const joints = this.solveFK(this.rightArm, baseRightX, baseRightY, scale);
                this.currentCube.x = joints.x2;
                this.currentCube.y = joints.y2;
            } else {
                const joints = this.solveFK(this.leftArm, baseLeftX, baseLeftY, scale);
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

    drawFarmingScene() {
        // Floor soil gradient (dark agtech greenhouse theme)
        const floorGrad = this.ctx.createLinearGradient(0, this.vpY, 0, this.logicalHeight);
        floorGrad.addColorStop(0, '#060B06');
        floorGrad.addColorStop(0.3, '#0A140B');
        floorGrad.addColorStop(1, '#112213');
        this.ctx.fillStyle = floorGrad;
        this.ctx.fillRect(0, this.vpY, this.logicalWidth, this.logicalHeight - this.vpY);

        const ceilingGrad = this.ctx.createLinearGradient(0, 0, 0, this.vpY);
        ceilingGrad.addColorStop(0, '#030503');
        ceilingGrad.addColorStop(1, '#060B06');
        this.ctx.fillStyle = ceilingGrad;
        this.ctx.fillRect(0, 0, this.logicalWidth, this.vpY);

        // Soil grid paths (Longitudinal)
        this.ctx.strokeStyle = '#162e1a';
        this.ctx.lineWidth = 1;
        
        const numGridLines = 10;
        for (let i = 0; i <= numGridLines; i++) {
            const t = i / numGridLines;
            const targetX = this.logicalWidth * t;
            this.ctx.beginPath();
            this.ctx.moveTo(this.vpX + (targetX - this.vpX) * 0.15, this.vpY);
            this.ctx.lineTo(targetX, this.logicalHeight);
            this.ctx.stroke();
        }

        // Soil paths (Transverse - exponential spacing)
        const numTransverse = 8;
        for (let i = 0; i <= numTransverse; i++) {
            const t = i / numTransverse;
            const y = this.vpY + (this.logicalHeight - this.vpY) * Math.pow(t, 2);
            this.ctx.beginPath();
            this.ctx.moveTo(0, y);
            this.ctx.lineTo(this.logicalWidth, y);
            this.ctx.stroke();
        }

        // Draw leafy crop rows/plants on Left and Right in perspective
        const drawCropsSide = (isLeft) => {
            const dir = isLeft ? -1 : 1;
            const zIntervals = [0.15, 0.35, 0.55, 0.75, 0.95];

            const plantX = (z) => this.vpX + dir * (35 + 230 * z);
            const floorY = (z) => this.vpY + (this.logicalHeight * 0.95 - this.vpY) * z;

            zIntervals.forEach((z, idx) => {
                const px = plantX(z);
                const py = floorY(z);
                const scale = z;

                // 1. Draw plant stalk/stem
                this.ctx.strokeStyle = '#451A03';
                this.ctx.lineWidth = 3.5 * scale;
                this.ctx.beginPath();
                this.ctx.moveTo(px, py);
                this.ctx.lineTo(px, py - 35 * scale);
                this.ctx.stroke();

                // 2. Draw green canopy leaf clumps
                this.ctx.fillStyle = '#15803D';
                this.ctx.beginPath();
                this.ctx.arc(px, py - 35 * scale, 15 * scale, 0, Math.PI * 2);
                this.ctx.arc(px - 10 * scale, py - 43 * scale, 11 * scale, 0, Math.PI * 2);
                this.ctx.arc(px + 10 * scale, py - 43 * scale, 11 * scale, 0, Math.PI * 2);
                this.ctx.fill();

                this.ctx.fillStyle = '#166534'; // Shadow overlay
                this.ctx.beginPath();
                this.ctx.arc(px - 5 * scale, py - 30 * scale, 8 * scale, 0, Math.PI * 2);
                this.ctx.arc(px + 5 * scale, py - 30 * scale, 8 * scale, 0, Math.PI * 2);
                this.ctx.fill();

                // 3. Draw static red fruits hanging (skip foreground)
                if (idx === zIntervals.length - 1) return;

                // Supply fruit is placed specifically in background Left
                if (isLeft && idx === 1 && this.state !== 1) {
                    // Handled in main draw loop
                    return;
                }

                this.ctx.fillStyle = this.colors.red;
                this.ctx.strokeStyle = '#FFFFFF';
                this.ctx.lineWidth = 0.5 * scale;
                
                // Hanging Fruit 1
                this.ctx.beginPath();
                this.ctx.arc(px - 6 * scale, py - 28 * scale, 3 * scale, 0, Math.PI * 2);
                this.ctx.fill();
                this.ctx.stroke();

                // Hanging Fruit 2
                this.ctx.beginPath();
                this.ctx.arc(px + 8 * scale, py - 34 * scale, 3 * scale, 0, Math.PI * 2);
                this.ctx.fill();
                this.ctx.stroke();
            });
        };

        drawCropsSide(true);
        drawCropsSide(false);

        // Smart hydroponic arched struts vaulting ceiling
        this.ctx.strokeStyle = 'rgba(6, 182, 212, 0.15)';
        this.ctx.lineWidth = 1;
        const arches = [0.2, 0.4, 0.6, 0.8];
        arches.forEach((z) => {
            const scale = z;
            const hY = this.vpY + (this.logicalHeight * 0.95 - this.vpY) * z;
            const wX = this.vpX;
            const rVal = (35 + 230 * z);

            this.ctx.beginPath();
            this.ctx.ellipse(wX, hY - 60 * scale, rVal, 100 * scale, 0, Math.PI, 0);
            this.ctx.stroke();
        });
    }

    drawWheel(cx, cy, r, angle, scale) {
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
        this.ctx.lineWidth = 1.5 * scale;
        this.ctx.beginPath();
        this.ctx.arc(0, 0, r - 3 * scale, 0, Math.PI * 2);
        this.ctx.stroke();

        // Spokes
        this.ctx.strokeStyle = this.colors.cyan;
        this.ctx.lineWidth = 1.0 * scale;
        for (let i = 0; i < 4; i++) {
            const spAngle = (i * Math.PI) / 2;
            this.ctx.beginPath();
            this.ctx.moveTo(0, 0);
            this.ctx.lineTo((r - 3 * scale) * Math.cos(spAngle), (r - 3 * scale) * Math.sin(spAngle));
            this.ctx.stroke();
        }

        // Hub Cap
        this.ctx.fillStyle = '#A8A29E';
        this.ctx.beginPath();
        this.ctx.arc(0, 0, 3 * scale, 0, Math.PI * 2);
        this.ctx.fill();

        this.ctx.restore();
    }

    drawRobot() {
        const scale = this.z;
        const floorY = this.vpY + (this.logicalHeight * 0.95 - this.vpY) * this.z;
        const chassisW = 56 * scale;
        const chassisH = 22 * scale;
        const chassisY = floorY - 14 * scale - chassisH;

        // 1. Draw Rear Wheels (behind chassis)
        const rearY = floorY - 16 * scale;
        this.drawWheel(this.vpX - 21 * scale, rearY, 7.5 * scale, this.wheelAngle, scale);
        this.drawWheel(this.vpX + 21 * scale, rearY, 7.5 * scale, this.wheelAngle, scale);

        // 2. Draw Chassis Body
        this.ctx.fillStyle = '#1E1B4B';
        this.ctx.strokeStyle = this.colors.purple;
        this.ctx.lineWidth = 1.5 * scale;
        
        this.ctx.beginPath();
        this.ctx.roundRect(this.vpX - chassisW/2, chassisY, chassisW, chassisH, 5 * scale);
        this.ctx.fill();
        this.ctx.stroke();

        // Glowing center core
        const corePulse = (2 + Math.abs(Math.sin(Date.now() / 150)) * 2) * scale;
        this.ctx.shadowColor = this.colors.cyan;
        this.ctx.shadowBlur = 8 * scale;
        this.ctx.fillStyle = this.colors.cyan;
        this.ctx.beginPath();
        this.ctx.arc(this.vpX, chassisY + chassisH/2, corePulse, 0, Math.PI * 2);
        this.ctx.fill();
        this.ctx.shadowBlur = 0; // Reset

        // Brand tag
        this.ctx.font = `700 ${Math.max(4, 7 * scale)}px monospace`;
        this.ctx.fillStyle = '#FFFFFF';
        this.ctx.textAlign = 'center';
        this.ctx.fillText('FEAR', this.vpX, chassisY + 8 * scale);

        // 3. Draw Front Wheels (in front of chassis)
        const frontY = floorY - 10 * scale;
        this.drawWheel(this.vpX - 26 * scale, frontY, 11 * scale, this.wheelAngle, scale);
        this.drawWheel(this.vpX + 26 * scale, frontY, 11 * scale, this.wheelAngle, scale);

        // 4. Draw Arms
        const armBaseY = chassisY + 4 * scale;
        this.drawArm(this.leftArm, this.vpX - 16 * scale, armBaseY, this.colors.cyan, scale);
        this.drawArm(this.rightArm, this.vpX + 16 * scale, armBaseY, this.colors.purple, scale);
    }

    drawArm(arm, baseX, baseY, accentColor, scale) {
        const joints = this.solveFK(arm, baseX, baseY, scale);

        // Shoulder joint hub
        this.ctx.fillStyle = '#09090B';
        this.ctx.strokeStyle = accentColor;
        this.ctx.lineWidth = 1.5 * scale;
        this.ctx.beginPath();
        this.ctx.arc(baseX, baseY, 6 * scale, 0, Math.PI * 2);
        this.ctx.fill();
        this.ctx.stroke();

        // Shoulder to Elbow Segment
        this.ctx.strokeStyle = '#3F3F46';
        this.ctx.lineWidth = 7 * scale;
        this.ctx.lineCap = 'round';
        this.ctx.beginPath();
        this.ctx.moveTo(baseX, baseY);
        this.ctx.lineTo(joints.x1, joints.y1);
        this.ctx.stroke();

        this.ctx.strokeStyle = accentColor;
        this.ctx.lineWidth = 1.5 * scale;
        this.ctx.beginPath();
        this.ctx.moveTo(baseX, baseY);
        this.ctx.lineTo(joints.x1, joints.y1);
        this.ctx.stroke();

        // Elbow Joint
        this.ctx.fillStyle = '#09090B';
        this.ctx.strokeStyle = accentColor;
        this.ctx.lineWidth = 1.5 * scale;
        this.ctx.beginPath();
        this.ctx.arc(joints.x1, joints.y1, 4 * scale, 0, Math.PI * 2);
        this.ctx.fill();
        this.ctx.stroke();

        // Elbow to Wrist Segment
        this.ctx.strokeStyle = '#27272A';
        this.ctx.lineWidth = 4.5 * scale;
        this.ctx.beginPath();
        this.ctx.moveTo(joints.x1, joints.y1);
        this.ctx.lineTo(joints.x2, joints.y2);
        this.ctx.stroke();

        this.ctx.strokeStyle = '#FFFFFF';
        this.ctx.lineWidth = 1 * scale;
        this.ctx.beginPath();
        this.ctx.moveTo(joints.x1, joints.y1);
        this.ctx.lineTo(joints.x2, joints.y2);
        this.ctx.stroke();

        // Gripper claws
        const handAngle = arm.theta1 + arm.theta2;
        const gripSize = 8 * scale;
        const fAngle = arm.gripperOpen ? 0.38 : 0.12;

        this.ctx.strokeStyle = accentColor;
        this.ctx.lineWidth = 2 * scale;
        
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

        // Draw round fruit shape
        this.ctx.fillStyle = color;
        this.ctx.shadowColor = color;
        this.ctx.shadowBlur = 6 * scale;
        this.ctx.beginPath();
        this.ctx.arc(0, 0, size/2, 0, Math.PI * 2);
        this.ctx.fill();

        // Fruit highlight highlight/reflection
        this.ctx.strokeStyle = '#FFFFFF';
        this.ctx.lineWidth = 1.0 * scale;
        this.ctx.shadowBlur = 0;
        this.ctx.stroke();

        // Draw small leaf/stalk on fruit
        this.ctx.strokeStyle = '#15803D';
        this.ctx.lineWidth = 1.5 * scale;
        this.ctx.beginPath();
        this.ctx.moveTo(0, -size/2);
        this.ctx.quadraticCurveTo(-2 * scale, -size/2 - 3 * scale, -4 * scale, -size/2 - 1 * scale);
        this.ctx.stroke();

        this.ctx.restore();
    }

    drawCrate(cx, cy, scale) {
        const w = 34 * scale;
        const h = 18 * scale;
        
        // Draw wood slats crate
        this.ctx.fillStyle = '#78350F'; // Wood brown
        this.ctx.strokeStyle = '#D97706'; // Amber outline
        this.ctx.lineWidth = 1.5 * scale;

        this.ctx.beginPath();
        this.ctx.rect(cx - w/2, cy, w, h);
        this.ctx.fill();
        this.ctx.stroke();

        // Horizontal slat divisions
        this.ctx.beginPath();
        this.ctx.moveTo(cx - w/2, cy + h * 0.35);
        this.ctx.lineTo(cx + w/2, cy + h * 0.35);
        this.ctx.moveTo(cx - w/2, cy + h * 0.7);
        this.ctx.lineTo(cx + w/2, cy + h * 0.7);
        this.ctx.strokeStyle = '#451A03';
        this.ctx.lineWidth = 1.0 * scale;
        this.ctx.stroke();

        // BRAND label
        this.ctx.fillStyle = 'rgba(255, 255, 255, 0.4)';
        this.ctx.font = `600 ${Math.max(3, 5 * scale)}px sans-serif`;
        this.ctx.textAlign = 'center';
        this.ctx.fillText('CROP', cx, cy + h - 3 * scale);
    }

    drawFruitsInCrate(cx, cy, count, scale) {
        const fr = 4.0 * scale;
        
        // Coordinates of fruits inside the box
        const positions = [
            { dx: -7 * scale, dy: 13 * scale },
            { dx: 6 * scale, dy: 13 * scale },
            { dx: -1 * scale, dy: 6 * scale }
        ];

        for (let i = 0; i < count; i++) {
            if (i >= positions.length) break;
            const px = cx + positions[i].dx;
            const py = cy + positions[i].dy;
            
            this.ctx.save();
            this.ctx.translate(px, py);
            this.ctx.fillStyle = this.colors.red;
            this.ctx.strokeStyle = '#FFFFFF';
            this.ctx.lineWidth = 0.8 * scale;
            this.ctx.beginPath();
            this.ctx.arc(0, 0, fr, 0, Math.PI * 2);
            this.ctx.fill();
            this.ctx.stroke();
            this.ctx.restore();
        }
    }

    draw() {
        this.ctx.clearRect(0, 0, this.logicalWidth, this.logicalHeight);

        // 1. Draw Farming Scene (soil, canopy row)
        this.drawFarmingScene();

        // 2. Draw Harvest Crate Bin (Right side foreground)
        const scaleDrop = this.zDrop;
        let crateOpacity = 1.0;
        if (this.state === 5 && this.stackCount >= 3) {
            crateOpacity = Math.max(0.3, 1.0 - (this.timer / 50));
        }
        this.ctx.globalAlpha = crateOpacity;
        this.drawCrate(this.boxX, this.boxY, scaleDrop);
        this.drawFruitsInCrate(this.boxX, this.boxY, this.stackCount, scaleDrop);
        this.ctx.globalAlpha = 1.0; // Reset

        // 3. Draw dust particles
        this.ctx.fillStyle = '#3F6212'; // green-ish dirt particles
        this.particles.forEach((p) => {
            this.ctx.globalAlpha = p.alpha;
            this.ctx.beginPath();
            this.ctx.arc(p.x, p.y, p.size, 0, Math.PI * 2);
            this.ctx.fill();
        });
        this.ctx.globalAlpha = 1.0; // Reset

        // 4. Draw Robot
        this.drawRobot();

        // 5. Draw current fruit carried by gripper
        if (this.currentCube) {
            this.drawCube(this.currentCube.x, this.currentCube.y, this.currentCube.color, 1.0, this.z);
        }

        // 6. Draw supply fruit hanging in background Left if not grabbed
        if (this.state !== 1) {
            this.drawCube(this.supplyX, this.supplyY, this.colors.red, 1.0, this.zPick);
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
