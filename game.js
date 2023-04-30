class Vector {
	constructor(x, y) {
		this.x = x;
		this.y = y;
	}

	translate(vector) {
		this.x += vector.x;
		this.y += vector.y;
	}

	addVector(vector) {
		this.x += vector.x;
		this.y += vector.y;
	}

	subtractVector(vector) {
		this.x -= vector.x;
		this.y -= vector.y;
	}

	addScalar(scalar) {
		this.x += scalar;
		this.y += scalar;
	}

	multiplyScalar(scalar) {
		this.x *= scalar;
		this.y *= scalar;
	}

	divideScalar(scalar) {
		this.x /= scalar;
		this.y /= scalar;
	}

	differenceVector(vector) {
		return new Vector(this.x - vector.x, this.y - vector.y);
	}

	dotProduct(vector) {
		return this.x * vector.x + this.y * vector.y;
	}

	vectorCrossProduct(vector) {
		return this.x * vector.y - this.y * vector.x;
	}

	scalarCrossProduct(scalar) {
		return new Vector(scalar * this.y, -scalar * this.x);
	}

	rotateAboutPoint(vector, angle) {
		let cos = Math.cos(angle);
		let sin = Math.sin(angle);

		let newX = (cos * (this.x - vector.x)) - (sin * (this.y - vector.y)) + vector.x;
		let newY = (cos * (this.y - vector.y)) + (sin * (this.x - vector.x)) + vector.y;

		this.x = newX;
		this.y = newY;
	}

	negate() {
		this.x = -this.x;
		this.y = -this.y;
	}

	normalize() {
		let distance = Math.sqrt(this.x * this.x + this.y * this.y);
		if (distance > 0) {
			this.x /= distance;
			this.y /= distance;
		}
	}

	set(x, y) {
		this.x = x;
		this.y = y;
	}

	copy() {
		return new Vector(this.x, this.y);
	}
}
let gravity = new Vector(0, 2);

class Body {
	constructor(vertices) {
		this.vertices = vertices;
		this.normals = [];
		this.density = 1;
		this.area = 1;
		this.angle = 0;
		this.mass = 1;
		this.invMass = 1;
		this.momentOfInertia = 1;
		this.invMomentOfInertia = 1/this.momentOfInertia;
		this.restitution = 0.4;
		this.friction = 0.1;
		this.red = 255;
		this.green = 0;
		this.blue = 0;
		this.alpha = 1;
		this.rivetedToBackground = false;

		this.velocity = new Vector(0, 0);
		this.force = new Vector(0, 0);

		this.angularVelocity = 0;
		this.torque = 0;

		this.computeArea();
		this.computeMass();

		if (this.vertices.length > 1) {
			this.computeNormals();
		}
	}

	applyForce(vector) {
		this.force.addVector(vector);
	}

	applyImpulse(impulse, contactVector) {
		let tempImpulse = impulse.copy();
		tempImpulse.multiplyScalar(this.invMass);
		this.velocity.addVector(tempImpulse);

		let deltaAngularVelocity = this.invMomentOfInertia * contactVector.vectorCrossProduct(impulse);
		this.angularVelocity += deltaAngularVelocity;
	}

	translate(vector) {
		for(var i=0; i<this.vertices.length; i++) {
			this.vertices[i].translate(vector);
		}
	}

	rotate(rotation) {
		if (this.vertices.length > 0) {
			if (this.vertices.length > 1) {
				let centerOfMass = this.findCenterOfMass();
				for (var i=0; i<this.vertices.length; i++) {
					this.vertices[i].rotateAboutPoint(centerOfMass, rotation);
				}
			}

			this.angle += rotation;
			this.angle = this.angle % (2*Math.PI);
			if (this.angle < 0) {
				this.angle += 2*Math.PI;
			}

			if (this.vertices.length > 1) {
				this.computeNormals();
			}
		}
	}

	getFarthestVertex(vector) {
		let farthestProjection = null;
		let farthestVertex = null;

		for (var i=0; i<this.vertices.length; i++) {
			let projection = this.vertices[i].dotProduct(vector);

			if (farthestProjection == null || projection > farthestProjection) {
				farthestProjection = projection;
				farthestVertex = this.vertices[i];
			}
		}

		return farthestVertex;
	}

	getLeastPenetration(body) {
		let mostDistance = null;
		let bestFace = null;

		for (var i=0; i<this.vertices.length; i++) {
			let normal = this.normals[i].copy();
			normal.negate();
			let farthestVertex = body.getFarthestVertex(normal).copy();
			farthestVertex.subtractVector(this.vertices[i]);
			normal.negate();

			let distance = normal.dotProduct(farthestVertex);
			if (mostDistance == null || distance > mostDistance) {
				mostDistance = distance;
				bestFace = i;
			}
		}

		return [mostDistance, bestFace];
	}

	findIncidentFace(incidentBody, referenceFace) {
		let incidentFace = null;
		let minimumDot = null;
		for (var i=0; i<incidentBody.vertices.length; i++) {
			let dot = this.normals[referenceFace].dotProduct(incidentBody.normals[i]);
			if (minimumDot == null || dot < minimumDot) {
				minimumDot = dot;
				incidentFace = i;
			}
		}

		return incidentFace;
	}

	findCenterOfMass() {
		let centerOfMass = new Vector(0, 0);
		for (var i=0; i<this.vertices.length; i++) {
			centerOfMass.addVector(this.vertices[i]);
		}
		centerOfMass.divideScalar(this.vertices.length);

		return centerOfMass;
	}

	computeArea() {
		this.area = 1;
	}

	computeMass() {
		this.mass = this.density * this.area;
		this.invMass = (this.mass == 0) ? 0 : 1/this.mass;
	}

	computeNormals() {
		this.normals = [];
		for (var i=0; i<this.vertices.length; i++) {
			let normalVector = this.vertices[(i+1) % this.vertices.length].copy();
			normalVector.subtractVector(this.vertices[i]);
			normalVector.normalize();

			let oldX = normalVector.x;
			normalVector.x = normalVector.y;
			normalVector.y = -oldX;

			this.normals.push(normalVector);
		}
	}

	setStatic() {
		this.mass = 0;
		this.invMass = 0;
		this.momentOfInertia = 0;
		this.invMomentOfInertia = 0;
	}

	isInside(vector) {
		return false;
	}

	tick(dt) {
		// a = F * 1/m
		if (this.mass > 0 && !this.rivetedToBackground) {
			let dtForce = this.force.copy();
			dtForce.multiplyScalar(this.invMass);
			dtForce.addVector(gravity);
			dtForce.multiplyScalar(dt);
			this.velocity.addVector(dtForce);
			this.translate(this.velocity);

			this.angularVelocity += this.torque * this.invMomentOfInertia * dt;
			if (this.angularVelocity != 0) {
				this.rotate(this.angularVelocity);
			}

			let airResistance = 0.9999;
			this.velocity.multiplyScalar(airResistance);
			this.angularVelocity *= airResistance;
		}

		this.force.set(0, 0);
		this.torque = 0;
	}

	render(context) {
		let canvas = context.canvas;
		context.strokeStyle = 'rgba(0, 0, 0, 1)';
		context.fillStyle = 'rgba(' + this.red + ', ' + this.green + ', ' + this.blue + ', ' + this.alpha + ')';
		context.lineWidth = 3;
		context.beginPath();
		context.moveTo(this.vertices[0].x, this.vertices[0].y);
		for (var i=1; i<this.vertices.length; i++) {
			context.lineTo(this.vertices[i].x, this.vertices[i].y);
		}
		context.lineTo(this.vertices[0].x, this.vertices[0].y);
		context.fill();
		context.stroke();
		context.closePath();
	}

	getName() {
		return 'Body';
	}
}

class Circle extends Body {
	constructor(center, radius) {
		super([center]);
		this.radius = radius;
		this.computeArea();
		this.computeMass();
		this.momentOfInertia = 4000000;
		this.invMomentOfInertia = 1/this.momentOfInertia;
	}

	computeArea() {
		if (this.radius) {
			this.area = Math.PI * this.radius * this.radius;
		}
	}

	isInside(vector) {
		return getDistance(vector, this.vertices[0]) < this.radius;
	}

	render(context) {
		let canvas = context.canvas;

		context.strokeStyle = 'rgba(0, 0, 0, 1)';
		context.fillStyle = 'rgba(' + this.red + ', ' + this.green + ', ' + this.blue + ', ' + this.alpha + ')';
		context.lineWidth = 3;
		context.beginPath();
		context.arc(this.vertices[0].x, this.vertices[0].y, this.radius, 0, 2*Math.PI, false);
		context.fill();
		context.moveTo(this.vertices[0].x, this.vertices[0].y);
		context.lineTo(this.vertices[0].x + Math.cos(this.angle) * this.radius, this.vertices[0].y + Math.sin(this.angle) * this.radius);
		context.stroke();
		context.closePath();
	}

	getName() {
		return 'Circle';
	}
}

class Rectangle extends Body {
	constructor(vertices) {
		super(vertices);
		this.momentOfInertia = 100000000;
		this.invMomentOfInertia = 1/this.momentOfInertia;
	}

	computeArea() {
		this.area = getDistance(this.vertices[0], this.vertices[1]) * getDistance(this.vertices[1], this.vertices[2]);
	}

	isInside(vector) {
		let tempArea = areaOfTriangle(this.vertices[0], vector, this.vertices[3]) + areaOfTriangle(this.vertices[3], vector, this.vertices[2]) + 
						areaOfTriangle(this.vertices[2], vector, this.vertices[1]) + areaOfTriangle(vector, this.vertices[1], this.vertices[0]);
		return Math.floor(tempArea) <= Math.floor(this.area);
	}

	getName() {
		return 'Rectangle';
	}
}

class Game {
	constructor() {
		this.bodies = [new Rectangle([new Vector(100, 1200), new Vector(1500, 1200), new Vector(1500, 1300), new Vector(100, 1300)]),
						new Circle(new Vector(200, 1100), 50)];
		this.bodies[0].setStatic();

		this.ticksPerSecond = 120;

		this.mousePosition = new Vector(0, 0);
		this.oldMousePosition = new Vector(0, 0);
		this.dragging = null;
		this.placing = null;
		this.placingDrag = false;
		this.draggingSlider = null;
		this.startTime = null;
		this.tickID = 0;
		this.dt = 1/this.ticksPerSecond;
		this.collisions = [];
		this.contextMenu = null;

		this.buildMenuY = 200;
		this.buildMenuWidth = 50;
		this.buildMenuHeight = 400;
		this.buildMenuCurveSize = 32;
		this.buildMenuIconSize = 48;

		this.contextMenuWidth = 300;
		this.contextMenuHeight = 400;
		this.contextMenuCurveSize = 16;
		this.contextMenuSliderWidth = 0.75 * this.contextMenuWidth;
		this.contextMenuSliderThickness = 3;
		this.contextMenuSliderGrabberWidth = 10;
		this.contextMenuSliderGrabberHeight = 14;
		this.contextMenuSliderGap = 45;
		this.sliders = ['angle', 'density', 'restitution', 'friction', 'red', 'green', 'blue', 'alpha'];
		this.slidersMin = [0, 0.1, 0, 0, 0, 0, 0, 0];
		this.slidersMax = [Math.round(2*Math.PI*100)/100, 10, 1, 1, 255, 255, 255, 1];

		this.canvas = document.createElement('canvas');
		this.canvas.width = window.innerWidth;
		this.canvas.height = window.innerHeight;
		this.canvas.oncontextmenu = function() {return false;};
		this.context = this.canvas.getContext('2d');
		this.context.imageSmoothingEnabled = false;
		this.context.mozImageSmoothingEnabled = false;
		this.context.webkitImageSmoothingEnabled = false;

		this.setupListeners();

		this.pause = true;
		this.canInteract = true;
		this.debug = false;
	}

	setupListeners() {
		let gameForListeners = this;
		addMouseDownListener(function(which, eventX, eventY) {
			switch(which) {
				case 1:
					let inContextMenu = false;
					if (gameForListeners.contextMenu != null) {
						let contextX = gameForListeners.contextMenu[1].x;
						let contextY = gameForListeners.contextMenu[1].y;
						let width = gameForListeners.contextMenuWidth;
						let height = gameForListeners.contextMenuHeight;
						let curveSize = gameForListeners.contextMenuCurveSize;

						if (eventX < contextX || eventX > contextX + 2*curveSize + width || eventY < contextY - 2*curveSize - height || eventY > contextY) {
							gameForListeners.contextMenu = null;
						} else {
							inContextMenu = true;
							if (eventX >= gameForListeners.contextMenu[1].x + gameForListeners.contextMenuCurveSize + (gameForListeners.contextMenuWidth - gameForListeners.contextMenuSliderWidth)/2 &&
								eventX <= gameForListeners.contextMenu[1].x + gameForListeners.contextMenuCurveSize + (gameForListeners.contextMenuWidth - gameForListeners.contextMenuSliderWidth)/2 + gameForListeners.contextMenuSliderWidth) {
								for (var i=0; i<gameForListeners.sliders.length; i++) {
									let sliderHeight = gameForListeners.contextMenuHeight - gameForListeners.contextMenuCurveSize - 22 - gameForListeners.contextMenuSliderGap * i;
									if (eventY >= gameForListeners.contextMenu[1].y - sliderHeight - gameForListeners.contextMenuSliderGrabberHeight - gameForListeners.contextMenuSliderThickness &&
										eventY <= gameForListeners.contextMenu[1].y - sliderHeight + gameForListeners.contextMenuSliderGrabberHeight + gameForListeners.contextMenuSliderThickness) {
										gameForListeners.draggingSlider = i;
										break;
									}
								}
							}
						}
					}

					if (gameForListeners.canInteract && !inContextMenu) {
						if (gameForListeners.placing != null) {
							gameForListeners.placingDrag = true;
						} else {
							if (eventX < gameForListeners.buildMenuWidth + gameForListeners.buildMenuCurveSize && 
								eventY < gameForListeners.buildMenuY + 2*gameForListeners.buildMenuCurveSize + gameForListeners.buildMenuHeight && eventY > gameForListeners.buildMenuY) {
								if (eventX < (gameForListeners.buildMenuWidth + gameForListeners.buildMenuCurveSize/1.25 + gameForListeners.buildMenuIconSize)/2 && 
									eventX > (gameForListeners.buildMenuWidth + gameForListeners.buildMenuCurveSize/1.25 - gameForListeners.buildMenuIconSize)/2) {
									for (var i=0; i<4; i++) {
										if (eventY < gameForListeners.buildMenuY + gameForListeners.buildMenuCurveSize + i*(gameForListeners.buildMenuIconSize + gameForListeners.buildMenuCurveSize) + gameForListeners.buildMenuIconSize && 
											eventY > gameForListeners.buildMenuY + gameForListeners.buildMenuCurveSize + i*(gameForListeners.buildMenuIconSize + gameForListeners.buildMenuCurveSize)) {
											switch(i) {
												case 0:
													gameForListeners.placing = new Circle(new Vector(eventX, eventY), 50);
													gameForListeners.placing.alpha = 0.5;
													break;
												case 1:
													gameForListeners.placing = new Rectangle([new Vector(eventX, eventY), new Vector(eventX + 200, eventY), new Vector(eventX + 200, eventY + 100), new Vector(eventX, eventY + 100)]);
													gameForListeners.placing.alpha = 0.5;
													break;
											}
										}
									}
								}
							} else {
								for (var i=gameForListeners.bodies.length-1; i>=0; i--) {
									if (gameForListeners.bodies[i].isInside(new Vector(eventX, eventY))) {
										gameForListeners.mousePosition.set(eventX, eventY);
										gameForListeners.dragging = gameForListeners.bodies[i];
										break;
									}
								}
							}
						}
					}
					break;
				case 3:
					gameForListeners.contextMenu = null;
					if (gameForListeners.canInteract && gameForListeners.dragging == null && gameForListeners.placing == null) {
						for (var i=gameForListeners.bodies.length-1; i>=0; i--) {
							if (gameForListeners.bodies[i].isInside(new Vector(eventX, eventY))) {
								gameForListeners.contextMenu = [gameForListeners.bodies[i], new Vector(eventX, eventY)];
								break;
							}
						}
					}
					break;
			}
		});

		addMouseUpListener(function(which, eventX, eventY) {
			switch(which) {
				case 1:
					gameForListeners.dragging = null;
					gameForListeners.draggingSlider = null;

					if (gameForListeners.placing != null && gameForListeners.placingDrag && getDistance(new Vector(eventX, eventY), gameForListeners.placing.vertices[0]) > 5) {
						gameForListeners.placing.computeArea();
						gameForListeners.placing.computeMass();
						if (gameForListeners.placing.vertices.length > 1) {
							gameForListeners.placing.computeNormals();
						}
						gameForListeners.placing.alpha = 1;

						gameForListeners.addBody(gameForListeners.placing);
						gameForListeners.placing = null;
						gameForListeners.placingDrag = false;
					}
					break;
			}
		});

		addMouseMoveListener(function(eventX, eventY) {
			gameForListeners.mousePosition.set(eventX, eventY);
		});

		addKeyDownListener(function(keyCode) {
			switch(keyCode) {
				case 'Space':
					gameForListeners.changePause();
					break;
			}
		});
	}

	addBody(body) {
		this.bodies.push(body);
	}

	changePause() {
		this.pause = !this.pause;
		this.canInteract = this.pause;

		if (!this.canInteract) {
			this.dragging = null;
			this.contextMenu = null;
			this.draggingSlider = null;
		}
	}

	setStartTime() {
		this.startTime = new Date();
	}

	timeSinceStart() {
		return new Date().getTime() - this.startTime;
	}

	tick() {
		this.tickID++;

		if (this.canInteract) {
			if (this.placing != null) {
				if (this.placingDrag) {
					if (this.placing instanceof Circle) {
						this.placing.radius = Math.round(getDistance(this.mousePosition, this.placing.vertices[0]));
					} else if (this.placing instanceof Rectangle) {
						let xDiff = this.mousePosition.x - this.placing.vertices[0].x;
						let yDiff = this.mousePosition.y - this.placing.vertices[0].y;
						if (xDiff * yDiff > 0) {
							this.placing.vertices[1].set(this.mousePosition.x, this.placing.vertices[0].y);
							this.placing.vertices[2].set(this.mousePosition.x, this.mousePosition.y);
							this.placing.vertices[3].set(this.placing.vertices[0].x, this.mousePosition.y);
						} else {
							this.placing.vertices[1].set(this.placing.vertices[0].x, this.mousePosition.y);
							this.placing.vertices[2].set(this.mousePosition.x, this.mousePosition.y);
							this.placing.vertices[3].set(this.mousePosition.x, this.placing.vertices[0].y);
						}
					}
				} else {
					this.placing.translate(this.mousePosition.differenceVector(this.oldMousePosition));
				}
			} else if (this.dragging != null) {
				this.dragging.translate(this.mousePosition.differenceVector(this.oldMousePosition));
			} else if (this.contextMenu != null && this.draggingSlider != null) {
				let sliderStartX = this.contextMenu[1].x + this.contextMenuCurveSize + (this.contextMenuWidth - this.contextMenuSliderWidth)/2;
				let percentage = (this.mousePosition.x - sliderStartX)/this.contextMenuSliderWidth;
				let actualNumber = Math.min(Math.max(percentage, 0), 1) * (this.slidersMax[this.draggingSlider] - this.slidersMin[this.draggingSlider]) + this.slidersMin[this.draggingSlider];
				if (Math.abs(this.slidersMax[this.draggingSlider]) <= 10) {
					actualNumber = Math.round(actualNumber * 1000)/1000;
				} else {
					actualNumber = Math.round(actualNumber);
				}

				if (this.sliders[this.draggingSlider] == 'angle') {
					this.contextMenu[0].rotate(actualNumber - this.contextMenu[0]['angle']);
				} else {
					this.contextMenu[0][this.sliders[this.draggingSlider]] = actualNumber;
				}
			}
		}

		if (!this.pause) {
			this.collisions = [];
			for (var i=0; i<this.bodies.length; i++) {
				for (var j=i+1; j<this.bodies.length; j++) {
					let collision = new Collision(this.bodies[i], this.bodies[j]);
					collision.solve();

					if (collision.contacts.length > 0) {
						this.collisions.push(collision);
					}
				}
			}

			for (var i=0; i<this.collisions.length; i++) {
				this.collisions[i].applyImpulse();
			}

			for (var i=0; i<this.bodies.length; i++) {
				this.bodies[i].tick(this.dt);
			}

			for (var i=0; i<this.collisions.length; i++) {
				this.collisions[i].positionalCorrection();
			}
		}

		this.oldMousePosition.set(this.mousePosition.x, this.mousePosition.y);
	}

	render() {
		this.context.fillStyle = 'rgba(115, 215, 255, 1)';
		this.context.fillRect(0, 0, this.canvas.width, this.canvas.height);

		for (var i=0; i<this.bodies.length; i++) {
			this.bodies[i].render(this.context);
		}

		this.context.fillStyle = 'rgba(40, 40, 40, 0.7)';
		this.context.strokeStyle = 'rgba(0, 0, 0, 1)';
		this.context.lineWidth = 1;
		this.context.beginPath();
		this.context.moveTo(0, this.buildMenuY - this.buildMenuCurveSize);
		this.context.lineTo(this.buildMenuWidth, this.buildMenuY - this.buildMenuCurveSize);
		this.context.arc(this.buildMenuWidth, this.buildMenuY, this.buildMenuCurveSize, 3*Math.PI/2, 2*Math.PI, false);
		this.context.lineTo(this.buildMenuWidth + this.buildMenuCurveSize, this.buildMenuY + this.buildMenuHeight + this.buildMenuCurveSize);
		this.context.arc(this.buildMenuWidth, this.buildMenuY + this.buildMenuHeight + this.buildMenuCurveSize, this.buildMenuCurveSize, 0, Math.PI/2, false);
		this.context.lineTo(0, this.buildMenuY + this.buildMenuHeight + 2*this.buildMenuCurveSize);
		this.context.fill();
		this.context.stroke();
		this.context.closePath();

		let buildMenuIcons = 0;

		this.context.fillStyle = 'rgba(255, 255, 255, 1)';
		this.context.fillRect((this.buildMenuWidth + this.buildMenuCurveSize/1.25 - this.buildMenuIconSize)/2, this.buildMenuY + this.buildMenuCurveSize, this.buildMenuIconSize, this.buildMenuIconSize);
		this.context.strokeRect((this.buildMenuWidth + this.buildMenuCurveSize/1.25 - this.buildMenuIconSize)/2, this.buildMenuY + this.buildMenuCurveSize, this.buildMenuIconSize, this.buildMenuIconSize);
		this.context.fillStyle = 'rgba(255, 0, 0, 1)';
		this.context.beginPath();
		this.context.arc((this.buildMenuWidth + this.buildMenuCurveSize/1.25)/2, this.buildMenuY + this.buildMenuCurveSize + this.buildMenuIconSize/2, this.buildMenuIconSize/3, 0, 2*Math.PI, false);
		this.context.fill();
		this.context.stroke();
		this.context.closePath();
		buildMenuIcons++;

		this.context.fillStyle = 'rgba(255, 255, 255, 1)';
		this.context.fillRect((this.buildMenuWidth + this.buildMenuCurveSize/1.25 - this.buildMenuIconSize)/2, this.buildMenuY + this.buildMenuCurveSize + buildMenuIcons*(this.buildMenuIconSize + this.buildMenuCurveSize), this.buildMenuIconSize, this.buildMenuIconSize);
		this.context.strokeRect((this.buildMenuWidth + this.buildMenuCurveSize/1.25 - this.buildMenuIconSize)/2, this.buildMenuY + this.buildMenuCurveSize + buildMenuIcons*(this.buildMenuIconSize + this.buildMenuCurveSize), this.buildMenuIconSize, this.buildMenuIconSize);
		this.context.fillStyle = 'rgba(255, 0, 0, 1)';
		this.context.fillRect((this.buildMenuWidth + this.buildMenuCurveSize/1.25 - this.buildMenuIconSize)/2 + this.buildMenuIconSize/10, this.buildMenuY + this.buildMenuCurveSize + this.buildMenuIconSize/4 + buildMenuIcons*(this.buildMenuIconSize + this.buildMenuCurveSize), 
								this.buildMenuIconSize*0.8, this.buildMenuIconSize*0.5);
		this.context.strokeRect((this.buildMenuWidth + this.buildMenuCurveSize/1.25 - this.buildMenuIconSize)/2 + this.buildMenuIconSize/10, this.buildMenuY + this.buildMenuCurveSize + this.buildMenuIconSize/4 + buildMenuIcons*(this.buildMenuIconSize + this.buildMenuCurveSize), 
								this.buildMenuIconSize*0.8, this.buildMenuIconSize*0.5);
		this.context.closePath();
		buildMenuIcons++;

		this.context.fillStyle = 'rgba(255, 255, 255, 1)';
		this.context.fillRect((this.buildMenuWidth + this.buildMenuCurveSize/1.25 - this.buildMenuIconSize)/2, this.buildMenuY + this.buildMenuCurveSize + buildMenuIcons*(this.buildMenuIconSize + this.buildMenuCurveSize), this.buildMenuIconSize, this.buildMenuIconSize);
		this.context.strokeRect((this.buildMenuWidth + this.buildMenuCurveSize/1.25 - this.buildMenuIconSize)/2, this.buildMenuY + this.buildMenuCurveSize + buildMenuIcons*(this.buildMenuIconSize + this.buildMenuCurveSize), this.buildMenuIconSize, this.buildMenuIconSize);
		this.context.fillStyle = 'rgba(0, 0, 255, 1)';
		this.context.beginPath();
		this.context.fill();
		this.context.stroke();
		this.context.closePath();
		buildMenuIcons++;

		if (this.contextMenu != null) {
			let top = this.contextMenu[1].y - 2*this.contextMenuCurveSize - this.contextMenuHeight;

			this.context.fillStyle = 'rgba(40, 40, 40, 0.7)';
			this.context.strokeStyle = 'rgba(0, 0, 0, 1)';
			this.context.lineWidth = 1;
			this.context.beginPath();
			this.context.moveTo(this.contextMenu[1].x + this.contextMenuCurveSize, this.contextMenu[1].y - 2*this.contextMenuCurveSize - this.contextMenuHeight);
			this.context.lineTo(this.contextMenu[1].x + this.contextMenuCurveSize + this.contextMenuWidth, this.contextMenu[1].y - 2*this.contextMenuCurveSize - this.contextMenuHeight);
			this.context.arc(this.contextMenu[1].x + this.contextMenuCurveSize + this.contextMenuWidth, this.contextMenu[1].y - this.contextMenuCurveSize - this.contextMenuHeight, this.contextMenuCurveSize, 3*Math.PI/2, 2*Math.PI, false);
			this.context.lineTo(this.contextMenu[1].x + 2*this.contextMenuCurveSize + this.contextMenuWidth, this.contextMenu[1].y - this.contextMenuCurveSize);
			this.context.arc(this.contextMenu[1].x + this.contextMenuCurveSize + this.contextMenuWidth, this.contextMenu[1].y - this.contextMenuCurveSize, this.contextMenuCurveSize, 0, Math.PI/2, false);
			this.context.lineTo(this.contextMenu[1].x + this.contextMenuCurveSize, this.contextMenu[1].y);
			this.context.arc(this.contextMenu[1].x + this.contextMenuCurveSize, this.contextMenu[1].y - this.contextMenuCurveSize, this.contextMenuCurveSize, Math.PI/2, Math.PI, false);
			this.context.lineTo(this.contextMenu[1].x, this.contextMenu[1].y - this.contextMenuCurveSize - this.contextMenuHeight);
			this.context.arc(this.contextMenu[1].x + this.contextMenuCurveSize, this.contextMenu[1].y - this.contextMenuCurveSize - this.contextMenuHeight, this.contextMenuCurveSize, Math.PI, 3*Math.PI/2, false);
			this.context.fill();
			this.context.stroke();
			this.context.closePath();

			this.context.font = 'bold 28px sans-serif';
			this.context.textAlign = 'left';
			this.context.textBaseline = 'top';
			this.context.fillStyle = 'rgba(190, 0, 0, 1)';
			this.context.fillRect(this.contextMenu[1].x + this.contextMenuCurveSize, top + this.contextMenuCurveSize + 22, this.contextMenuWidth, 3);

			this.context.fillStyle = 'rgba(255, 50, 50, 1)';
			this.context.lineWidth = 1;
			this.context.fillText(this.contextMenu[0].getName(), this.contextMenu[1].x + this.contextMenuCurveSize, top + this.contextMenuCurveSize - 2);

			let sliderHeight = 0;
			for (var j=0; j<this.sliders.length; j++) {
				let variable = this.contextMenu[0][this.sliders[j]];
				sliderHeight = this.contextMenuHeight - this.contextMenuCurveSize - 22 - this.contextMenuSliderGap * j;
				this.context.font = '12px sans-serif';
				this.context.fillStyle = 'rgba(0, 0, 0, 1)';
				this.context.strokeStyle = 'rgba(0, 0, 0, 1)';
				this.context.textAlign = 'center';
				this.context.textBaseline = 'alphabetic';
				this.context.fillText(this.sliders[j][0].toUpperCase() + this.sliders[j].slice(1), this.contextMenu[1].x + this.contextMenuCurveSize + (this.contextMenuWidth - this.contextMenuSliderWidth)/2, this.contextMenu[1].y - sliderHeight - 10);
				this.context.textBaseline = 'top';
				this.context.fillText(this.slidersMin[j], this.contextMenu[1].x + this.contextMenuCurveSize + (this.contextMenuWidth - this.contextMenuSliderWidth)/2, this.contextMenu[1].y - sliderHeight + 8);
				this.context.fillText(this.slidersMax[j], this.contextMenu[1].x + this.contextMenuCurveSize + (this.contextMenuWidth - this.contextMenuSliderWidth)/2 + this.contextMenuSliderWidth, this.contextMenu[1].y - sliderHeight + 8);
				this.context.fillStyle = 'rgba(220, 220, 220, 1)';
				this.context.fillText(variable, this.contextMenu[1].x + this.contextMenuCurveSize + (this.contextMenuWidth - this.contextMenuSliderWidth)/2 + this.contextMenuSliderWidth/2, this.contextMenu[1].y - sliderHeight + 8);
				this.context.fillStyle = 'rgba(0, 0, 0, 1)';
				this.context.fillRect(this.contextMenu[1].x + this.contextMenuCurveSize + (this.contextMenuWidth - this.contextMenuSliderWidth)/2, this.contextMenu[1].y - sliderHeight, this.contextMenuSliderWidth, this.contextMenuSliderThickness);
				this.context.fillStyle = 'rgba(255, 255, 255, 1)';
				this.context.lineWidth = 1;
				this.context.beginPath();
				this.context.moveTo(Math.round(this.contextMenu[1].x + this.contextMenuCurveSize + (this.contextMenuWidth - this.contextMenuSliderWidth)/2 + ((variable - this.slidersMin[j])/(this.slidersMax[j] - this.slidersMin[j])) * this.contextMenuSliderWidth - this.contextMenuSliderGrabberWidth/2), 
									Math.round(this.contextMenu[1].y - sliderHeight + 2 - this.contextMenuSliderGrabberHeight/2));
				this.context.lineTo(Math.round(this.contextMenu[1].x + this.contextMenuCurveSize + (this.contextMenuWidth - this.contextMenuSliderWidth)/2 + ((variable - this.slidersMin[j])/(this.slidersMax[j] - this.slidersMin[j])) * this.contextMenuSliderWidth + this.contextMenuSliderGrabberWidth/2), 
									Math.round(this.contextMenu[1].y - sliderHeight + 2 - this.contextMenuSliderGrabberHeight/2));
				this.context.lineTo(Math.round(this.contextMenu[1].x + this.contextMenuCurveSize + (this.contextMenuWidth - this.contextMenuSliderWidth)/2 + ((variable - this.slidersMin[j])/(this.slidersMax[j] - this.slidersMin[j])) * this.contextMenuSliderWidth + this.contextMenuSliderGrabberWidth/2), 
									Math.round(this.contextMenu[1].y - sliderHeight + 2));
				this.context.lineTo(Math.round(this.contextMenu[1].x + this.contextMenuCurveSize + (this.contextMenuWidth - this.contextMenuSliderWidth)/2 + ((variable - this.slidersMin[j])/(this.slidersMax[j] - this.slidersMin[j])) * this.contextMenuSliderWidth), 
									Math.round(this.contextMenu[1].y - sliderHeight + 2 + this.contextMenuSliderGrabberHeight/2));
				this.context.lineTo(Math.round(this.contextMenu[1].x + this.contextMenuCurveSize + (this.contextMenuWidth - this.contextMenuSliderWidth)/2 + ((variable - this.slidersMin[j])/(this.slidersMax[j] - this.slidersMin[j])) * this.contextMenuSliderWidth - this.contextMenuSliderGrabberWidth/2), 
									Math.round(this.contextMenu[1].y - sliderHeight + 2));
				this.context.lineTo(Math.round(this.contextMenu[1].x + this.contextMenuCurveSize + (this.contextMenuWidth - this.contextMenuSliderWidth)/2 + ((variable - this.slidersMin[j])/(this.slidersMax[j] - this.slidersMin[j])) * this.contextMenuSliderWidth - this.contextMenuSliderGrabberWidth/2), 
									Math.round(this.contextMenu[1].y - sliderHeight + 2 - this.contextMenuSliderGrabberHeight/2));
				this.context.fill();
				this.context.stroke();
				this.context.closePath();
			}
		}

		if (this.placing != null) {
			this.placing.render(this.context);
		}

		if (this.debug) {
			for (var i=0; i<this.collisions.length; i++) {
				for (var j=0; j<this.collisions[i].contacts.length; j++) {
					this.context.strokeStyle = 'rgba(0, 255, 0, 1)';
					this.context.lineWidth = 5;
					this.context.beginPath();
					this.context.arc(this.collisions[i].contacts[j].x, this.collisions[i].contacts[j].y, 10, 0, 2*Math.PI, false);
					this.context.stroke();
					this.context.closePath();
				}
			}
		}
	}
}

let totalTimeUnloaded = 0;
let mostRecentUnload = null;
document.addEventListener('visibilitychange', function () {
	if (document.visibilityState === 'visible' && mostRecentUnload) {
		totalTimeUnloaded += (new Date().getTime() - mostRecentUnload.getTime());
		mostRecentUnload = null;
	} else if (document.visibilityState === 'hidden' && !mostRecentUnload) {
		mostRecentUnload = new Date();
	}
});

function gameLoop(game) {
	while (game.timeSinceStart() - totalTimeUnloaded >= (1000/game.ticksPerSecond) * game.tickID) {
		game.tick();
	}
	game.render();

	window.requestAnimationFrame(function() {gameLoop(game);});
}

function start() {
	let game = new Game();
	game.setStartTime();
	window.requestAnimationFrame(function() {gameLoop(game);});
	window.onload = function() {
		document.body.appendChild(game.canvas);
	}
}

start();