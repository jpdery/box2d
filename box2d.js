(function(modules) {
    var cache = {}, require = function(id) {
        var module = cache[id];
        if (!module) {
            module = cache[id] = {};
            var exports = module.exports = {};
            modules[id].call(exports, require, module, exports, window);
        }
        return module.exports;
    };
    window["box2d"] = require("0");
})({
    "0": function(require, module, exports, global) {
        var box2d = {};
        box2d.Vec2 = require("1");
        box2d.BodyDef = require("2");
        box2d.Body = require("3");
        box2d.FixtureDef = require("4");
        box2d.Fixture = require("5");
        box2d.World = require("6");
        box2d.MassData = require("7");
        box2d.PolygonShape = require("8");
        box2d.CircleShape = require("9");
        box2d.DebugDraw = require("a");
        module.export = box2d;
    },
    "1": function(require, module, exports, global) {
        var b2Vec2 = function(x_, y_) {
            if (arguments.length == 2) {
                this.x = x_;
                this.y = y_;
            }
        };
        b2Vec2.Make = function(x_, y_) {
            return new b2Vec2(x_, y_);
        };
        b2Vec2.prototype.SetZero = function() {
            this.x = 0;
            this.y = 0;
        };
        b2Vec2.prototype.Set = function(x_, y_) {
            this.x = x_;
            this.y = y_;
        };
        b2Vec2.prototype.SetV = function(v) {
            this.x = v.x;
            this.y = v.y;
        };
        b2Vec2.prototype.GetNegative = function() {
            return new b2Vec2(-this.x, -this.y);
        };
        b2Vec2.prototype.NegativeSelf = function() {
            this.x = -this.x;
            this.y = -this.y;
        };
        b2Vec2.prototype.Copy = function() {
            return new b2Vec2(this.x, this.y);
        };
        b2Vec2.prototype.Add = function(v) {
            this.x += v.x;
            this.y += v.y;
        };
        b2Vec2.prototype.Subtract = function(v) {
            this.x -= v.x;
            this.y -= v.y;
        };
        b2Vec2.prototype.Multiply = function(a) {
            this.x *= a;
            this.y *= a;
        };
        b2Vec2.prototype.MulM = function(A) {
            var tX = this.x;
            this.x = A.col1.x * tX + A.col2.x * this.y;
            this.y = A.col1.y * tX + A.col2.y * this.y;
        };
        b2Vec2.prototype.MulTM = function(A) {
            var tX = b2Math.Dot(this, A.col1);
            this.y = b2Math.Dot(this, A.col2);
            this.x = tX;
        };
        b2Vec2.prototype.CrossVF = function(s) {
            var tX = this.x;
            this.x = s * this.y;
            this.y = -s * tX;
        };
        b2Vec2.prototype.CrossFV = function(s) {
            var tX = this.x;
            this.x = -s * this.y;
            this.y = s * tX;
        };
        b2Vec2.prototype.MinV = function(b) {
            this.x = this.x < b.x ? this.x : b.x;
            this.y = this.y < b.y ? this.y : b.y;
        };
        b2Vec2.prototype.MaxV = function(b) {
            this.x = this.x > b.x ? this.x : b.x;
            this.y = this.y > b.y ? this.y : b.y;
        };
        b2Vec2.prototype.Abs = function() {
            if (this.x < 0) this.x = -this.x;
            if (this.y < 0) this.y = -this.y;
        };
        b2Vec2.prototype.Length = function() {
            return Math.sqrt(this.x * this.x + this.y * this.y);
        };
        b2Vec2.prototype.LengthSquared = function() {
            return this.x * this.x + this.y * this.y;
        };
        b2Vec2.prototype.Normalize = function() {
            var length = Math.sqrt(this.x * this.x + this.y * this.y);
            if (length < Number.MIN_VALUE) {
                return 0;
            }
            var invLength = 1 / length;
            this.x *= invLength;
            this.y *= invLength;
            return length;
        };
        b2Vec2.prototype.IsValid = function() {
            return b2Math.IsValid(this.x) && b2Math.IsValid(this.y);
        };
        b2Vec2.prototype.x = 0;
        b2Vec2.prototype.y = 0;
        module.exports = b2Vec2;
    },
    "2": function(require, module, exports, global) {
        var b2BodyDef = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2BodyDef.prototype.__constructor = function() {
            this.userData = null;
            this.position.Set(0, 0);
            this.angle = 0;
            this.linearVelocity.Set(0, 0);
            this.angularVelocity = 0;
            this.linearDamping = 0;
            this.angularDamping = 0;
            this.allowSleep = true;
            this.awake = true;
            this.fixedRotation = false;
            this.bullet = false;
            this.type = b2Body.b2_staticBody;
            this.active = true;
            this.inertiaScale = 1;
        };
        b2BodyDef.prototype.__varz = function() {
            this.position = new b2Vec2;
            this.linearVelocity = new b2Vec2;
        };
        b2BodyDef.prototype.type = 0;
        b2BodyDef.prototype.position = new b2Vec2;
        b2BodyDef.prototype.angle = null;
        b2BodyDef.prototype.linearVelocity = new b2Vec2;
        b2BodyDef.prototype.angularVelocity = null;
        b2BodyDef.prototype.linearDamping = null;
        b2BodyDef.prototype.angularDamping = null;
        b2BodyDef.prototype.allowSleep = null;
        b2BodyDef.prototype.awake = null;
        b2BodyDef.prototype.fixedRotation = null;
        b2BodyDef.prototype.bullet = null;
        b2BodyDef.prototype.active = null;
        b2BodyDef.prototype.userData = null;
        b2BodyDef.prototype.inertiaScale = null;
        module.exports = b2BodyDef;
    },
    "3": function(require, module, exports, global) {
        var b2Body = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2Body.prototype.__constructor = function(bd, world) {
            this.m_flags = 0;
            if (bd.bullet) {
                this.m_flags |= b2Body.e_bulletFlag;
            }
            if (bd.fixedRotation) {
                this.m_flags |= b2Body.e_fixedRotationFlag;
            }
            if (bd.allowSleep) {
                this.m_flags |= b2Body.e_allowSleepFlag;
            }
            if (bd.awake) {
                this.m_flags |= b2Body.e_awakeFlag;
            }
            if (bd.active) {
                this.m_flags |= b2Body.e_activeFlag;
            }
            this.m_world = world;
            this.m_xf.position.SetV(bd.position);
            this.m_xf.R.Set(bd.angle);
            this.m_sweep.localCenter.SetZero();
            this.m_sweep.t0 = 1;
            this.m_sweep.a0 = this.m_sweep.a = bd.angle;
            var tMat = this.m_xf.R;
            var tVec = this.m_sweep.localCenter;
            this.m_sweep.c.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            this.m_sweep.c.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            this.m_sweep.c.x += this.m_xf.position.x;
            this.m_sweep.c.y += this.m_xf.position.y;
            this.m_sweep.c0.SetV(this.m_sweep.c);
            this.m_jointList = null;
            this.m_controllerList = null;
            this.m_contactList = null;
            this.m_controllerCount = 0;
            this.m_prev = null;
            this.m_next = null;
            this.m_linearVelocity.SetV(bd.linearVelocity);
            this.m_angularVelocity = bd.angularVelocity;
            this.m_linearDamping = bd.linearDamping;
            this.m_angularDamping = bd.angularDamping;
            this.m_force.Set(0, 0);
            this.m_torque = 0;
            this.m_sleepTime = 0;
            this.m_type = bd.type;
            if (this.m_type == b2Body.b2_dynamicBody) {
                this.m_mass = 1;
                this.m_invMass = 1;
            } else {
                this.m_mass = 0;
                this.m_invMass = 0;
            }
            this.m_I = 0;
            this.m_invI = 0;
            this.m_inertiaScale = bd.inertiaScale;
            this.m_userData = bd.userData;
            this.m_fixtureList = null;
            this.m_fixtureCount = 0;
        };
        b2Body.prototype.__varz = function() {
            this.m_xf = new b2Transform;
            this.m_sweep = new b2Sweep;
            this.m_linearVelocity = new b2Vec2;
            this.m_force = new b2Vec2;
        };
        b2Body.b2_staticBody = 0;
        b2Body.b2_kinematicBody = 1;
        b2Body.b2_dynamicBody = 2;
        b2Body.s_xf1 = new b2Transform;
        b2Body.e_islandFlag = 1;
        b2Body.e_awakeFlag = 2;
        b2Body.e_allowSleepFlag = 4;
        b2Body.e_bulletFlag = 8;
        b2Body.e_fixedRotationFlag = 16;
        b2Body.e_activeFlag = 32;
        b2Body.prototype.connectEdges = function(s1, s2, angle1) {
            var angle2 = Math.atan2(s2.GetDirectionVector().y, s2.GetDirectionVector().x);
            var coreOffset = Math.tan((angle2 - angle1) * .5);
            var core = b2Math.MulFV(coreOffset, s2.GetDirectionVector());
            core = b2Math.SubtractVV(core, s2.GetNormalVector());
            core = b2Math.MulFV(b2Settings.b2_toiSlop, core);
            core = b2Math.AddVV(core, s2.GetVertex1());
            var cornerDir = b2Math.AddVV(s1.GetDirectionVector(), s2.GetDirectionVector());
            cornerDir.Normalize();
            var convex = b2Math.Dot(s1.GetDirectionVector(), s2.GetNormalVector()) > 0;
            s1.SetNextEdge(s2, core, cornerDir, convex);
            s2.SetPrevEdge(s1, core, cornerDir, convex);
            return angle2;
        };
        b2Body.prototype.SynchronizeFixtures = function() {
            var xf1 = b2Body.s_xf1;
            xf1.R.Set(this.m_sweep.a0);
            var tMat = xf1.R;
            var tVec = this.m_sweep.localCenter;
            xf1.position.x = this.m_sweep.c0.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            xf1.position.y = this.m_sweep.c0.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            var f;
            var broadPhase = this.m_world.m_contactManager.m_broadPhase;
            for (f = this.m_fixtureList; f; f = f.m_next) {
                f.Synchronize(broadPhase, xf1, this.m_xf);
            }
        };
        b2Body.prototype.SynchronizeTransform = function() {
            this.m_xf.R.Set(this.m_sweep.a);
            var tMat = this.m_xf.R;
            var tVec = this.m_sweep.localCenter;
            this.m_xf.position.x = this.m_sweep.c.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            this.m_xf.position.y = this.m_sweep.c.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
        };
        b2Body.prototype.ShouldCollide = function(other) {
            if (this.m_type != b2Body.b2_dynamicBody && other.m_type != b2Body.b2_dynamicBody) {
                return false;
            }
            for (var jn = this.m_jointList; jn; jn = jn.next) {
                if (jn.other == other) if (jn.joint.m_collideConnected == false) {
                    return false;
                }
            }
            return true;
        };
        b2Body.prototype.Advance = function(t) {
            this.m_sweep.Advance(t);
            this.m_sweep.c.SetV(this.m_sweep.c0);
            this.m_sweep.a = this.m_sweep.a0;
            this.SynchronizeTransform();
        };
        b2Body.prototype.CreateFixture = function(def) {
            if (this.m_world.IsLocked() == true) {
                return null;
            }
            var fixture = new b2Fixture;
            fixture.Create(this, this.m_xf, def);
            if (this.m_flags & b2Body.e_activeFlag) {
                var broadPhase = this.m_world.m_contactManager.m_broadPhase;
                fixture.CreateProxy(broadPhase, this.m_xf);
            }
            fixture.m_next = this.m_fixtureList;
            this.m_fixtureList = fixture;
            ++this.m_fixtureCount;
            fixture.m_body = this;
            if (fixture.m_density > 0) {
                this.ResetMassData();
            }
            this.m_world.m_flags |= b2World.e_newFixture;
            return fixture;
        };
        b2Body.prototype.CreateFixture2 = function(shape, density) {
            var def = new b2FixtureDef;
            def.shape = shape;
            def.density = density;
            return this.CreateFixture(def);
        };
        b2Body.prototype.DestroyFixture = function(fixture) {
            if (this.m_world.IsLocked() == true) {
                return;
            }
            var node = this.m_fixtureList;
            var ppF = null;
            var found = false;
            while (node != null) {
                if (node == fixture) {
                    if (ppF) ppF.m_next = fixture.m_next; else this.m_fixtureList = fixture.m_next;
                    found = true;
                    break;
                }
                ppF = node;
                node = node.m_next;
            }
            var edge = this.m_contactList;
            while (edge) {
                var c = edge.contact;
                edge = edge.next;
                var fixtureA = c.GetFixtureA();
                var fixtureB = c.GetFixtureB();
                if (fixture == fixtureA || fixture == fixtureB) {
                    this.m_world.m_contactManager.Destroy(c);
                }
            }
            if (this.m_flags & b2Body.e_activeFlag) {
                var broadPhase = this.m_world.m_contactManager.m_broadPhase;
                fixture.DestroyProxy(broadPhase);
            } else {}
            fixture.Destroy();
            fixture.m_body = null;
            fixture.m_next = null;
            --this.m_fixtureCount;
            this.ResetMassData();
        };
        b2Body.prototype.SetPositionAndAngle = function(position, angle) {
            var f;
            if (this.m_world.IsLocked() == true) {
                return;
            }
            this.m_xf.R.Set(angle);
            this.m_xf.position.SetV(position);
            var tMat = this.m_xf.R;
            var tVec = this.m_sweep.localCenter;
            this.m_sweep.c.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
            this.m_sweep.c.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
            this.m_sweep.c.x += this.m_xf.position.x;
            this.m_sweep.c.y += this.m_xf.position.y;
            this.m_sweep.c0.SetV(this.m_sweep.c);
            this.m_sweep.a0 = this.m_sweep.a = angle;
            var broadPhase = this.m_world.m_contactManager.m_broadPhase;
            for (f = this.m_fixtureList; f; f = f.m_next) {
                f.Synchronize(broadPhase, this.m_xf, this.m_xf);
            }
            this.m_world.m_contactManager.FindNewContacts();
        };
        b2Body.prototype.SetTransform = function(xf) {
            this.SetPositionAndAngle(xf.position, xf.GetAngle());
        };
        b2Body.prototype.GetTransform = function() {
            return this.m_xf;
        };
        b2Body.prototype.GetPosition = function() {
            return this.m_xf.position;
        };
        b2Body.prototype.SetPosition = function(position) {
            this.SetPositionAndAngle(position, this.GetAngle());
        };
        b2Body.prototype.GetAngle = function() {
            return this.m_sweep.a;
        };
        b2Body.prototype.SetAngle = function(angle) {
            this.SetPositionAndAngle(this.GetPosition(), angle);
        };
        b2Body.prototype.GetWorldCenter = function() {
            return this.m_sweep.c;
        };
        b2Body.prototype.GetLocalCenter = function() {
            return this.m_sweep.localCenter;
        };
        b2Body.prototype.SetLinearVelocity = function(v) {
            if (this.m_type == b2Body.b2_staticBody) {
                return;
            }
            this.m_linearVelocity.SetV(v);
        };
        b2Body.prototype.GetLinearVelocity = function() {
            return this.m_linearVelocity;
        };
        b2Body.prototype.SetAngularVelocity = function(omega) {
            if (this.m_type == b2Body.b2_staticBody) {
                return;
            }
            this.m_angularVelocity = omega;
        };
        b2Body.prototype.GetAngularVelocity = function() {
            return this.m_angularVelocity;
        };
        b2Body.prototype.GetDefinition = function() {
            var bd = new b2BodyDef;
            bd.type = this.GetType();
            bd.allowSleep = (this.m_flags & b2Body.e_allowSleepFlag) == b2Body.e_allowSleepFlag;
            bd.angle = this.GetAngle();
            bd.angularDamping = this.m_angularDamping;
            bd.angularVelocity = this.m_angularVelocity;
            bd.fixedRotation = (this.m_flags & b2Body.e_fixedRotationFlag) == b2Body.e_fixedRotationFlag;
            bd.bullet = (this.m_flags & b2Body.e_bulletFlag) == b2Body.e_bulletFlag;
            bd.awake = (this.m_flags & b2Body.e_awakeFlag) == b2Body.e_awakeFlag;
            bd.linearDamping = this.m_linearDamping;
            bd.linearVelocity.SetV(this.GetLinearVelocity());
            bd.position = this.GetPosition();
            bd.userData = this.GetUserData();
            return bd;
        };
        b2Body.prototype.ApplyForce = function(force, point) {
            if (this.m_type != b2Body.b2_dynamicBody) {
                return;
            }
            if (this.IsAwake() == false) {
                this.SetAwake(true);
            }
            this.m_force.x += force.x;
            this.m_force.y += force.y;
            this.m_torque += (point.x - this.m_sweep.c.x) * force.y - (point.y - this.m_sweep.c.y) * force.x;
        };
        b2Body.prototype.ApplyTorque = function(torque) {
            if (this.m_type != b2Body.b2_dynamicBody) {
                return;
            }
            if (this.IsAwake() == false) {
                this.SetAwake(true);
            }
            this.m_torque += torque;
        };
        b2Body.prototype.ApplyImpulse = function(impulse, point) {
            if (this.m_type != b2Body.b2_dynamicBody) {
                return;
            }
            if (this.IsAwake() == false) {
                this.SetAwake(true);
            }
            this.m_linearVelocity.x += this.m_invMass * impulse.x;
            this.m_linearVelocity.y += this.m_invMass * impulse.y;
            this.m_angularVelocity += this.m_invI * ((point.x - this.m_sweep.c.x) * impulse.y - (point.y - this.m_sweep.c.y) * impulse.x);
        };
        b2Body.prototype.Split = function(callback) {
            var linearVelocity = this.GetLinearVelocity().Copy();
            var angularVelocity = this.GetAngularVelocity();
            var center = this.GetWorldCenter();
            var body1 = this;
            var body2 = this.m_world.CreateBody(this.GetDefinition());
            var prev;
            for (var f = body1.m_fixtureList; f; ) {
                if (callback(f)) {
                    var next = f.m_next;
                    if (prev) {
                        prev.m_next = next;
                    } else {
                        body1.m_fixtureList = next;
                    }
                    body1.m_fixtureCount--;
                    f.m_next = body2.m_fixtureList;
                    body2.m_fixtureList = f;
                    body2.m_fixtureCount++;
                    f.m_body = body2;
                    f = next;
                } else {
                    prev = f;
                    f = f.m_next;
                }
            }
            body1.ResetMassData();
            body2.ResetMassData();
            var center1 = body1.GetWorldCenter();
            var center2 = body2.GetWorldCenter();
            var velocity1 = b2Math.AddVV(linearVelocity, b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center1, center)));
            var velocity2 = b2Math.AddVV(linearVelocity, b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center2, center)));
            body1.SetLinearVelocity(velocity1);
            body2.SetLinearVelocity(velocity2);
            body1.SetAngularVelocity(angularVelocity);
            body2.SetAngularVelocity(angularVelocity);
            body1.SynchronizeFixtures();
            body2.SynchronizeFixtures();
            return body2;
        };
        b2Body.prototype.Merge = function(other) {
            var f;
            for (f = other.m_fixtureList; f; ) {
                var next = f.m_next;
                other.m_fixtureCount--;
                f.m_next = this.m_fixtureList;
                this.m_fixtureList = f;
                this.m_fixtureCount++;
                f.m_body = body2;
                f = next;
            }
            body1.m_fixtureCount = 0;
            var body1 = this;
            var body2 = other;
            var center1 = body1.GetWorldCenter();
            var center2 = body2.GetWorldCenter();
            var velocity1 = body1.GetLinearVelocity().Copy();
            var velocity2 = body2.GetLinearVelocity().Copy();
            var angular1 = body1.GetAngularVelocity();
            var angular = body2.GetAngularVelocity();
            body1.ResetMassData();
            this.SynchronizeFixtures();
        };
        b2Body.prototype.GetMass = function() {
            return this.m_mass;
        };
        b2Body.prototype.GetInertia = function() {
            return this.m_I;
        };
        b2Body.prototype.GetMassData = function(data) {
            data.mass = this.m_mass;
            data.I = this.m_I;
            data.center.SetV(this.m_sweep.localCenter);
        };
        b2Body.prototype.SetMassData = function(massData) {
            b2Settings.b2Assert(this.m_world.IsLocked() == false);
            if (this.m_world.IsLocked() == true) {
                return;
            }
            if (this.m_type != b2Body.b2_dynamicBody) {
                return;
            }
            this.m_invMass = 0;
            this.m_I = 0;
            this.m_invI = 0;
            this.m_mass = massData.mass;
            if (this.m_mass <= 0) {
                this.m_mass = 1;
            }
            this.m_invMass = 1 / this.m_mass;
            if (massData.I > 0 && (this.m_flags & b2Body.e_fixedRotationFlag) == 0) {
                this.m_I = massData.I - this.m_mass * (massData.center.x * massData.center.x + massData.center.y * massData.center.y);
                this.m_invI = 1 / this.m_I;
            }
            var oldCenter = this.m_sweep.c.Copy();
            this.m_sweep.localCenter.SetV(massData.center);
            this.m_sweep.c0.SetV(b2Math.MulX(this.m_xf, this.m_sweep.localCenter));
            this.m_sweep.c.SetV(this.m_sweep.c0);
            this.m_linearVelocity.x += this.m_angularVelocity * -(this.m_sweep.c.y - oldCenter.y);
            this.m_linearVelocity.y += this.m_angularVelocity * +(this.m_sweep.c.x - oldCenter.x);
        };
        b2Body.prototype.ResetMassData = function() {
            this.m_mass = 0;
            this.m_invMass = 0;
            this.m_I = 0;
            this.m_invI = 0;
            this.m_sweep.localCenter.SetZero();
            if (this.m_type == b2Body.b2_staticBody || this.m_type == b2Body.b2_kinematicBody) {
                return;
            }
            var center = b2Vec2.Make(0, 0);
            for (var f = this.m_fixtureList; f; f = f.m_next) {
                if (f.m_density == 0) {
                    continue;
                }
                var massData = f.GetMassData();
                this.m_mass += massData.mass;
                center.x += massData.center.x * massData.mass;
                center.y += massData.center.y * massData.mass;
                this.m_I += massData.I;
            }
            if (this.m_mass > 0) {
                this.m_invMass = 1 / this.m_mass;
                center.x *= this.m_invMass;
                center.y *= this.m_invMass;
            } else {
                this.m_mass = 1;
                this.m_invMass = 1;
            }
            if (this.m_I > 0 && (this.m_flags & b2Body.e_fixedRotationFlag) == 0) {
                this.m_I -= this.m_mass * (center.x * center.x + center.y * center.y);
                this.m_I *= this.m_inertiaScale;
                b2Settings.b2Assert(this.m_I > 0);
                this.m_invI = 1 / this.m_I;
            } else {
                this.m_I = 0;
                this.m_invI = 0;
            }
            var oldCenter = this.m_sweep.c.Copy();
            this.m_sweep.localCenter.SetV(center);
            this.m_sweep.c0.SetV(b2Math.MulX(this.m_xf, this.m_sweep.localCenter));
            this.m_sweep.c.SetV(this.m_sweep.c0);
            this.m_linearVelocity.x += this.m_angularVelocity * -(this.m_sweep.c.y - oldCenter.y);
            this.m_linearVelocity.y += this.m_angularVelocity * +(this.m_sweep.c.x - oldCenter.x);
        };
        b2Body.prototype.GetWorldPoint = function(localPoint) {
            var A = this.m_xf.R;
            var u = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y);
            u.x += this.m_xf.position.x;
            u.y += this.m_xf.position.y;
            return u;
        };
        b2Body.prototype.GetWorldVector = function(localVector) {
            return b2Math.MulMV(this.m_xf.R, localVector);
        };
        b2Body.prototype.GetLocalPoint = function(worldPoint) {
            return b2Math.MulXT(this.m_xf, worldPoint);
        };
        b2Body.prototype.GetLocalVector = function(worldVector) {
            return b2Math.MulTMV(this.m_xf.R, worldVector);
        };
        b2Body.prototype.GetLinearVelocityFromWorldPoint = function(worldPoint) {
            return new b2Vec2(this.m_linearVelocity.x - this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y), this.m_linearVelocity.y + this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x));
        };
        b2Body.prototype.GetLinearVelocityFromLocalPoint = function(localPoint) {
            var A = this.m_xf.R;
            var worldPoint = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y);
            worldPoint.x += this.m_xf.position.x;
            worldPoint.y += this.m_xf.position.y;
            return new b2Vec2(this.m_linearVelocity.x - this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y), this.m_linearVelocity.y + this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x));
        };
        b2Body.prototype.GetLinearDamping = function() {
            return this.m_linearDamping;
        };
        b2Body.prototype.SetLinearDamping = function(linearDamping) {
            this.m_linearDamping = linearDamping;
        };
        b2Body.prototype.GetAngularDamping = function() {
            return this.m_angularDamping;
        };
        b2Body.prototype.SetAngularDamping = function(angularDamping) {
            this.m_angularDamping = angularDamping;
        };
        b2Body.prototype.SetType = function(type) {
            if (this.m_type == type) {
                return;
            }
            this.m_type = type;
            this.ResetMassData();
            if (this.m_type == b2Body.b2_staticBody) {
                this.m_linearVelocity.SetZero();
                this.m_angularVelocity = 0;
            }
            this.SetAwake(true);
            this.m_force.SetZero();
            this.m_torque = 0;
            for (var ce = this.m_contactList; ce; ce = ce.next) {
                ce.contact.FlagForFiltering();
            }
        };
        b2Body.prototype.GetType = function() {
            return this.m_type;
        };
        b2Body.prototype.SetBullet = function(flag) {
            if (flag) {
                this.m_flags |= b2Body.e_bulletFlag;
            } else {
                this.m_flags &= ~b2Body.e_bulletFlag;
            }
        };
        b2Body.prototype.IsBullet = function() {
            return (this.m_flags & b2Body.e_bulletFlag) == b2Body.e_bulletFlag;
        };
        b2Body.prototype.SetSleepingAllowed = function(flag) {
            if (flag) {
                this.m_flags |= b2Body.e_allowSleepFlag;
            } else {
                this.m_flags &= ~b2Body.e_allowSleepFlag;
                this.SetAwake(true);
            }
        };
        b2Body.prototype.SetAwake = function(flag) {
            if (flag) {
                this.m_flags |= b2Body.e_awakeFlag;
                this.m_sleepTime = 0;
            } else {
                this.m_flags &= ~b2Body.e_awakeFlag;
                this.m_sleepTime = 0;
                this.m_linearVelocity.SetZero();
                this.m_angularVelocity = 0;
                this.m_force.SetZero();
                this.m_torque = 0;
            }
        };
        b2Body.prototype.IsAwake = function() {
            return (this.m_flags & b2Body.e_awakeFlag) == b2Body.e_awakeFlag;
        };
        b2Body.prototype.SetFixedRotation = function(fixed) {
            if (fixed) {
                this.m_flags |= b2Body.e_fixedRotationFlag;
            } else {
                this.m_flags &= ~b2Body.e_fixedRotationFlag;
            }
            this.ResetMassData();
        };
        b2Body.prototype.IsFixedRotation = function() {
            return (this.m_flags & b2Body.e_fixedRotationFlag) == b2Body.e_fixedRotationFlag;
        };
        b2Body.prototype.SetActive = function(flag) {
            if (flag == this.IsActive()) {
                return;
            }
            var broadPhase;
            var f;
            if (flag) {
                this.m_flags |= b2Body.e_activeFlag;
                broadPhase = this.m_world.m_contactManager.m_broadPhase;
                for (f = this.m_fixtureList; f; f = f.m_next) {
                    f.CreateProxy(broadPhase, this.m_xf);
                }
            } else {
                this.m_flags &= ~b2Body.e_activeFlag;
                broadPhase = this.m_world.m_contactManager.m_broadPhase;
                for (f = this.m_fixtureList; f; f = f.m_next) {
                    f.DestroyProxy(broadPhase);
                }
                var ce = this.m_contactList;
                while (ce) {
                    var ce0 = ce;
                    ce = ce.next;
                    this.m_world.m_contactManager.Destroy(ce0.contact);
                }
                this.m_contactList = null;
            }
        };
        b2Body.prototype.IsActive = function() {
            return (this.m_flags & b2Body.e_activeFlag) == b2Body.e_activeFlag;
        };
        b2Body.prototype.IsSleepingAllowed = function() {
            return (this.m_flags & b2Body.e_allowSleepFlag) == b2Body.e_allowSleepFlag;
        };
        b2Body.prototype.GetFixtureList = function() {
            return this.m_fixtureList;
        };
        b2Body.prototype.GetJointList = function() {
            return this.m_jointList;
        };
        b2Body.prototype.GetControllerList = function() {
            return this.m_controllerList;
        };
        b2Body.prototype.GetContactList = function() {
            return this.m_contactList;
        };
        b2Body.prototype.GetNext = function() {
            return this.m_next;
        };
        b2Body.prototype.GetUserData = function() {
            return this.m_userData;
        };
        b2Body.prototype.SetUserData = function(data) {
            this.m_userData = data;
        };
        b2Body.prototype.GetWorld = function() {
            return this.m_world;
        };
        b2Body.prototype.m_flags = 0;
        b2Body.prototype.m_type = 0;
        b2Body.prototype.m_islandIndex = 0;
        b2Body.prototype.m_xf = new b2Transform;
        b2Body.prototype.m_sweep = new b2Sweep;
        b2Body.prototype.m_linearVelocity = new b2Vec2;
        b2Body.prototype.m_angularVelocity = null;
        b2Body.prototype.m_force = new b2Vec2;
        b2Body.prototype.m_torque = null;
        b2Body.prototype.m_world = null;
        b2Body.prototype.m_prev = null;
        b2Body.prototype.m_next = null;
        b2Body.prototype.m_fixtureList = null;
        b2Body.prototype.m_fixtureCount = 0;
        b2Body.prototype.m_controllerList = null;
        b2Body.prototype.m_controllerCount = 0;
        b2Body.prototype.m_jointList = null;
        b2Body.prototype.m_contactList = null;
        b2Body.prototype.m_mass = null;
        b2Body.prototype.m_invMass = null;
        b2Body.prototype.m_I = null;
        b2Body.prototype.m_invI = null;
        b2Body.prototype.m_inertiaScale = null;
        b2Body.prototype.m_linearDamping = null;
        b2Body.prototype.m_angularDamping = null;
        b2Body.prototype.m_sleepTime = null;
        b2Body.prototype.m_userData = null;
        module.exports = b2Body;
    },
    "4": function(require, module, exports, global) {
        var b2FixtureDef = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2FixtureDef.prototype.__constructor = function() {
            this.shape = null;
            this.userData = null;
            this.friction = .2;
            this.restitution = 0;
            this.density = 0;
            this.filter.categoryBits = 1;
            this.filter.maskBits = 65535;
            this.filter.groupIndex = 0;
            this.isSensor = false;
        };
        b2FixtureDef.prototype.__varz = function() {
            this.filter = new b2FilterData;
        };
        b2FixtureDef.prototype.shape = null;
        b2FixtureDef.prototype.userData = null;
        b2FixtureDef.prototype.friction = null;
        b2FixtureDef.prototype.restitution = null;
        b2FixtureDef.prototype.density = null;
        b2FixtureDef.prototype.isSensor = null;
        b2FixtureDef.prototype.filter = new b2FilterData;
        module.exports = b2FixtureDef;
    },
    "5": function(require, module, exports, global) {
        var b2Fixture = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2Fixture.prototype.__constructor = function() {
            this.m_aabb = new b2AABB;
            this.m_userData = null;
            this.m_body = null;
            this.m_next = null;
            this.m_shape = null;
            this.m_density = 0;
            this.m_friction = 0;
            this.m_restitution = 0;
        };
        b2Fixture.prototype.__varz = function() {
            this.m_filter = new b2FilterData;
        };
        b2Fixture.prototype.Create = function(body, xf, def) {
            this.m_userData = def.userData;
            this.m_friction = def.friction;
            this.m_restitution = def.restitution;
            this.m_body = body;
            this.m_next = null;
            this.m_filter = def.filter.Copy();
            this.m_isSensor = def.isSensor;
            this.m_shape = def.shape.Copy();
            this.m_density = def.density;
        };
        b2Fixture.prototype.Destroy = function() {
            this.m_shape = null;
        };
        b2Fixture.prototype.CreateProxy = function(broadPhase, xf) {
            this.m_shape.ComputeAABB(this.m_aabb, xf);
            this.m_proxy = broadPhase.CreateProxy(this.m_aabb, this);
        };
        b2Fixture.prototype.DestroyProxy = function(broadPhase) {
            if (this.m_proxy == null) {
                return;
            }
            broadPhase.DestroyProxy(this.m_proxy);
            this.m_proxy = null;
        };
        b2Fixture.prototype.Synchronize = function(broadPhase, transform1, transform2) {
            if (!this.m_proxy) return;
            var aabb1 = new b2AABB;
            var aabb2 = new b2AABB;
            this.m_shape.ComputeAABB(aabb1, transform1);
            this.m_shape.ComputeAABB(aabb2, transform2);
            this.m_aabb.Combine(aabb1, aabb2);
            var displacement = b2Math.SubtractVV(transform2.position, transform1.position);
            broadPhase.MoveProxy(this.m_proxy, this.m_aabb, displacement);
        };
        b2Fixture.prototype.GetType = function() {
            return this.m_shape.GetType();
        };
        b2Fixture.prototype.GetShape = function() {
            return this.m_shape;
        };
        b2Fixture.prototype.SetSensor = function(sensor) {
            if (this.m_isSensor == sensor) return;
            this.m_isSensor = sensor;
            if (this.m_body == null) return;
            var edge = this.m_body.GetContactList();
            while (edge) {
                var contact = edge.contact;
                var fixtureA = contact.GetFixtureA();
                var fixtureB = contact.GetFixtureB();
                if (fixtureA == this || fixtureB == this) contact.SetSensor(fixtureA.IsSensor() || fixtureB.IsSensor());
                edge = edge.next;
            }
        };
        b2Fixture.prototype.IsSensor = function() {
            return this.m_isSensor;
        };
        b2Fixture.prototype.SetFilterData = function(filter) {
            this.m_filter = filter.Copy();
            if (this.m_body) return;
            var edge = this.m_body.GetContactList();
            while (edge) {
                var contact = edge.contact;
                var fixtureA = contact.GetFixtureA();
                var fixtureB = contact.GetFixtureB();
                if (fixtureA == this || fixtureB == this) contact.FlagForFiltering();
                edge = edge.next;
            }
        };
        b2Fixture.prototype.GetFilterData = function() {
            return this.m_filter.Copy();
        };
        b2Fixture.prototype.GetBody = function() {
            return this.m_body;
        };
        b2Fixture.prototype.GetNext = function() {
            return this.m_next;
        };
        b2Fixture.prototype.GetUserData = function() {
            return this.m_userData;
        };
        b2Fixture.prototype.SetUserData = function(data) {
            this.m_userData = data;
        };
        b2Fixture.prototype.TestPoint = function(p) {
            return this.m_shape.TestPoint(this.m_body.GetTransform(), p);
        };
        b2Fixture.prototype.RayCast = function(output, input) {
            return this.m_shape.RayCast(output, input, this.m_body.GetTransform());
        };
        b2Fixture.prototype.GetMassData = function(massData) {
            if (massData == null) {
                massData = new b2MassData;
            }
            this.m_shape.ComputeMass(massData, this.m_density);
            return massData;
        };
        b2Fixture.prototype.SetDensity = function(density) {
            this.m_density = density;
        };
        b2Fixture.prototype.GetDensity = function() {
            return this.m_density;
        };
        b2Fixture.prototype.GetFriction = function() {
            return this.m_friction;
        };
        b2Fixture.prototype.SetFriction = function(friction) {
            this.m_friction = friction;
        };
        b2Fixture.prototype.GetRestitution = function() {
            return this.m_restitution;
        };
        b2Fixture.prototype.SetRestitution = function(restitution) {
            this.m_restitution = restitution;
        };
        b2Fixture.prototype.GetAABB = function() {
            return this.m_aabb;
        };
        b2Fixture.prototype.m_massData = null;
        b2Fixture.prototype.m_aabb = null;
        b2Fixture.prototype.m_density = null;
        b2Fixture.prototype.m_next = null;
        b2Fixture.prototype.m_body = null;
        b2Fixture.prototype.m_shape = null;
        b2Fixture.prototype.m_friction = null;
        b2Fixture.prototype.m_restitution = null;
        b2Fixture.prototype.m_proxy = null;
        b2Fixture.prototype.m_filter = new b2FilterData;
        b2Fixture.prototype.m_isSensor = null;
        b2Fixture.prototype.m_userData = null;
        module.exports = b2Fixture;
    },
    "6": function(require, module, exports, global) {
        var b2World = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2World.prototype.__constructor = function(gravity, doSleep) {
            this.m_destructionListener = null;
            this.m_debugDraw = null;
            this.m_bodyList = null;
            this.m_contactList = null;
            this.m_jointList = null;
            this.m_controllerList = null;
            this.m_bodyCount = 0;
            this.m_contactCount = 0;
            this.m_jointCount = 0;
            this.m_controllerCount = 0;
            b2World.m_warmStarting = true;
            b2World.m_continuousPhysics = true;
            this.m_allowSleep = doSleep;
            this.m_gravity = gravity;
            this.m_inv_dt0 = 0;
            this.m_contactManager.m_world = this;
            var bd = new b2BodyDef;
            this.m_groundBody = this.CreateBody(bd);
        };
        b2World.prototype.__varz = function() {
            this.s_stack = new Array;
            this.m_contactManager = new b2ContactManager;
            this.m_contactSolver = new b2ContactSolver;
            this.m_island = new b2Island;
        };
        b2World.s_timestep2 = new b2TimeStep;
        b2World.s_backupA = new b2Sweep;
        b2World.s_backupB = new b2Sweep;
        b2World.s_timestep = new b2TimeStep;
        b2World.s_queue = new Array;
        b2World.e_newFixture = 1;
        b2World.e_locked = 2;
        b2World.s_xf = new b2Transform;
        b2World.s_jointColor = new b2Color(.5, .8, .8);
        b2World.m_warmStarting = null;
        b2World.m_continuousPhysics = null;
        b2World.prototype.Solve = function(step) {
            var b;
            for (var controller = this.m_controllerList; controller; controller = controller.m_next) {
                controller.Step(step);
            }
            var island = this.m_island;
            island.Initialize(this.m_bodyCount, this.m_contactCount, this.m_jointCount, null, this.m_contactManager.m_contactListener, this.m_contactSolver);
            for (b = this.m_bodyList; b; b = b.m_next) {
                b.m_flags &= ~b2Body.e_islandFlag;
            }
            for (var c = this.m_contactList; c; c = c.m_next) {
                c.m_flags &= ~b2Contact.e_islandFlag;
            }
            for (var j = this.m_jointList; j; j = j.m_next) {
                j.m_islandFlag = false;
            }
            var stackSize = this.m_bodyCount;
            var stack = this.s_stack;
            for (var seed = this.m_bodyList; seed; seed = seed.m_next) {
                if (seed.m_flags & b2Body.e_islandFlag) {
                    continue;
                }
                if (seed.IsAwake() == false || seed.IsActive() == false) {
                    continue;
                }
                if (seed.GetType() == b2Body.b2_staticBody) {
                    continue;
                }
                island.Clear();
                var stackCount = 0;
                stack[stackCount++] = seed;
                seed.m_flags |= b2Body.e_islandFlag;
                while (stackCount > 0) {
                    b = stack[--stackCount];
                    island.AddBody(b);
                    if (b.IsAwake() == false) {
                        b.SetAwake(true);
                    }
                    if (b.GetType() == b2Body.b2_staticBody) {
                        continue;
                    }
                    var other;
                    for (var ce = b.m_contactList; ce; ce = ce.next) {
                        if (ce.contact.m_flags & b2Contact.e_islandFlag) {
                            continue;
                        }
                        if (ce.contact.IsSensor() == true || ce.contact.IsEnabled() == false || ce.contact.IsTouching() == false) {
                            continue;
                        }
                        island.AddContact(ce.contact);
                        ce.contact.m_flags |= b2Contact.e_islandFlag;
                        other = ce.other;
                        if (other.m_flags & b2Body.e_islandFlag) {
                            continue;
                        }
                        stack[stackCount++] = other;
                        other.m_flags |= b2Body.e_islandFlag;
                    }
                    for (var jn = b.m_jointList; jn; jn = jn.next) {
                        if (jn.joint.m_islandFlag == true) {
                            continue;
                        }
                        other = jn.other;
                        if (other.IsActive() == false) {
                            continue;
                        }
                        island.AddJoint(jn.joint);
                        jn.joint.m_islandFlag = true;
                        if (other.m_flags & b2Body.e_islandFlag) {
                            continue;
                        }
                        stack[stackCount++] = other;
                        other.m_flags |= b2Body.e_islandFlag;
                    }
                }
                island.Solve(step, this.m_gravity, this.m_allowSleep);
                for (var i = 0; i < island.m_bodyCount; ++i) {
                    b = island.m_bodies[i];
                    if (b.GetType() == b2Body.b2_staticBody) {
                        b.m_flags &= ~b2Body.e_islandFlag;
                    }
                }
            }
            for (i = 0; i < stack.length; ++i) {
                if (!stack[i]) break;
                stack[i] = null;
            }
            for (b = this.m_bodyList; b; b = b.m_next) {
                if (b.IsAwake() == false || b.IsActive() == false) {
                    continue;
                }
                if (b.GetType() == b2Body.b2_staticBody) {
                    continue;
                }
                b.SynchronizeFixtures();
            }
            this.m_contactManager.FindNewContacts();
        };
        b2World.prototype.SolveTOI = function(step) {
            var b;
            var fA;
            var fB;
            var bA;
            var bB;
            var cEdge;
            var j;
            var island = this.m_island;
            island.Initialize(this.m_bodyCount, b2Settings.b2_maxTOIContactsPerIsland, b2Settings.b2_maxTOIJointsPerIsland, null, this.m_contactManager.m_contactListener, this.m_contactSolver);
            var queue = b2World.s_queue;
            for (b = this.m_bodyList; b; b = b.m_next) {
                b.m_flags &= ~b2Body.e_islandFlag;
                b.m_sweep.t0 = 0;
            }
            var c;
            for (c = this.m_contactList; c; c = c.m_next) {
                c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
            }
            for (j = this.m_jointList; j; j = j.m_next) {
                j.m_islandFlag = false;
            }
            for (;;) {
                var minContact = null;
                var minTOI = 1;
                for (c = this.m_contactList; c; c = c.m_next) {
                    if (c.IsSensor() == true || c.IsEnabled() == false || c.IsContinuous() == false) {
                        continue;
                    }
                    var toi = 1;
                    if (c.m_flags & b2Contact.e_toiFlag) {
                        toi = c.m_toi;
                    } else {
                        fA = c.m_fixtureA;
                        fB = c.m_fixtureB;
                        bA = fA.m_body;
                        bB = fB.m_body;
                        if ((bA.GetType() != b2Body.b2_dynamicBody || bA.IsAwake() == false) && (bB.GetType() != b2Body.b2_dynamicBody || bB.IsAwake() == false)) {
                            continue;
                        }
                        var t0 = bA.m_sweep.t0;
                        if (bA.m_sweep.t0 < bB.m_sweep.t0) {
                            t0 = bB.m_sweep.t0;
                            bA.m_sweep.Advance(t0);
                        } else if (bB.m_sweep.t0 < bA.m_sweep.t0) {
                            t0 = bA.m_sweep.t0;
                            bB.m_sweep.Advance(t0);
                        }
                        toi = c.ComputeTOI(bA.m_sweep, bB.m_sweep);
                        b2Settings.b2Assert(0 <= toi && toi <= 1);
                        if (toi > 0 && toi < 1) {
                            toi = (1 - toi) * t0 + toi;
                            if (toi > 1) toi = 1;
                        }
                        c.m_toi = toi;
                        c.m_flags |= b2Contact.e_toiFlag;
                    }
                    if (Number.MIN_VALUE < toi && toi < minTOI) {
                        minContact = c;
                        minTOI = toi;
                    }
                }
                if (minContact == null || 1 - 100 * Number.MIN_VALUE < minTOI) {
                    break;
                }
                fA = minContact.m_fixtureA;
                fB = minContact.m_fixtureB;
                bA = fA.m_body;
                bB = fB.m_body;
                b2World.s_backupA.Set(bA.m_sweep);
                b2World.s_backupB.Set(bB.m_sweep);
                bA.Advance(minTOI);
                bB.Advance(minTOI);
                minContact.Update(this.m_contactManager.m_contactListener);
                minContact.m_flags &= ~b2Contact.e_toiFlag;
                if (minContact.IsSensor() == true || minContact.IsEnabled() == false) {
                    bA.m_sweep.Set(b2World.s_backupA);
                    bB.m_sweep.Set(b2World.s_backupB);
                    bA.SynchronizeTransform();
                    bB.SynchronizeTransform();
                    continue;
                }
                if (minContact.IsTouching() == false) {
                    continue;
                }
                var seed = bA;
                if (seed.GetType() != b2Body.b2_dynamicBody) {
                    seed = bB;
                }
                island.Clear();
                var queueStart = 0;
                var queueSize = 0;
                queue[queueStart + queueSize++] = seed;
                seed.m_flags |= b2Body.e_islandFlag;
                while (queueSize > 0) {
                    b = queue[queueStart++];
                    --queueSize;
                    island.AddBody(b);
                    if (b.IsAwake() == false) {
                        b.SetAwake(true);
                    }
                    if (b.GetType() != b2Body.b2_dynamicBody) {
                        continue;
                    }
                    for (cEdge = b.m_contactList; cEdge; cEdge = cEdge.next) {
                        if (island.m_contactCount == island.m_contactCapacity) {
                            break;
                        }
                        if (cEdge.contact.m_flags & b2Contact.e_islandFlag) {
                            continue;
                        }
                        if (cEdge.contact.IsSensor() == true || cEdge.contact.IsEnabled() == false || cEdge.contact.IsTouching() == false) {
                            continue;
                        }
                        island.AddContact(cEdge.contact);
                        cEdge.contact.m_flags |= b2Contact.e_islandFlag;
                        var other = cEdge.other;
                        if (other.m_flags & b2Body.e_islandFlag) {
                            continue;
                        }
                        if (other.GetType() != b2Body.b2_staticBody) {
                            other.Advance(minTOI);
                            other.SetAwake(true);
                        }
                        queue[queueStart + queueSize] = other;
                        ++queueSize;
                        other.m_flags |= b2Body.e_islandFlag;
                    }
                    for (var jEdge = b.m_jointList; jEdge; jEdge = jEdge.next) {
                        if (island.m_jointCount == island.m_jointCapacity) continue;
                        if (jEdge.joint.m_islandFlag == true) continue;
                        other = jEdge.other;
                        if (other.IsActive() == false) {
                            continue;
                        }
                        island.AddJoint(jEdge.joint);
                        jEdge.joint.m_islandFlag = true;
                        if (other.m_flags & b2Body.e_islandFlag) continue;
                        if (other.GetType() != b2Body.b2_staticBody) {
                            other.Advance(minTOI);
                            other.SetAwake(true);
                        }
                        queue[queueStart + queueSize] = other;
                        ++queueSize;
                        other.m_flags |= b2Body.e_islandFlag;
                    }
                }
                var subStep = b2World.s_timestep;
                subStep.warmStarting = false;
                subStep.dt = (1 - minTOI) * step.dt;
                subStep.inv_dt = 1 / subStep.dt;
                subStep.dtRatio = 0;
                subStep.velocityIterations = step.velocityIterations;
                subStep.positionIterations = step.positionIterations;
                island.SolveTOI(subStep);
                var i = 0;
                for (i = 0; i < island.m_bodyCount; ++i) {
                    b = island.m_bodies[i];
                    b.m_flags &= ~b2Body.e_islandFlag;
                    if (b.IsAwake() == false) {
                        continue;
                    }
                    if (b.GetType() != b2Body.b2_dynamicBody) {
                        continue;
                    }
                    b.SynchronizeFixtures();
                    for (cEdge = b.m_contactList; cEdge; cEdge = cEdge.next) {
                        cEdge.contact.m_flags &= ~b2Contact.e_toiFlag;
                    }
                }
                for (i = 0; i < island.m_contactCount; ++i) {
                    c = island.m_contacts[i];
                    c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
                }
                for (i = 0; i < island.m_jointCount; ++i) {
                    j = island.m_joints[i];
                    j.m_islandFlag = false;
                }
                this.m_contactManager.FindNewContacts();
            }
        };
        b2World.prototype.DrawJoint = function(joint) {
            var b1 = joint.GetBodyA();
            var b2 = joint.GetBodyB();
            var xf1 = b1.m_xf;
            var xf2 = b2.m_xf;
            var x1 = xf1.position;
            var x2 = xf2.position;
            var p1 = joint.GetAnchorA();
            var p2 = joint.GetAnchorB();
            var color = b2World.s_jointColor;
            switch (joint.m_type) {
              case b2Joint.e_distanceJoint:
                this.m_debugDraw.DrawSegment(p1, p2, color);
                break;
              case b2Joint.e_pulleyJoint:
                {
                    var pulley = joint;
                    var s1 = pulley.GetGroundAnchorA();
                    var s2 = pulley.GetGroundAnchorB();
                    this.m_debugDraw.DrawSegment(s1, p1, color);
                    this.m_debugDraw.DrawSegment(s2, p2, color);
                    this.m_debugDraw.DrawSegment(s1, s2, color);
                }
                break;
              case b2Joint.e_mouseJoint:
                this.m_debugDraw.DrawSegment(p1, p2, color);
                break;
              default:
                if (b1 != this.m_groundBody) this.m_debugDraw.DrawSegment(x1, p1, color);
                this.m_debugDraw.DrawSegment(p1, p2, color);
                if (b2 != this.m_groundBody) this.m_debugDraw.DrawSegment(x2, p2, color);
            }
        };
        b2World.prototype.DrawShape = function(shape, xf, color) {
            switch (shape.m_type) {
              case b2Shape.e_circleShape:
                {
                    var circle = shape;
                    var center = b2Math.MulX(xf, circle.m_p);
                    var radius = circle.m_radius;
                    var axis = xf.R.col1;
                    this.m_debugDraw.DrawSolidCircle(center, radius, axis, color);
                }
                break;
              case b2Shape.e_polygonShape:
                {
                    var i = 0;
                    var poly = shape;
                    var vertexCount = poly.GetVertexCount();
                    var localVertices = poly.GetVertices();
                    var vertices = new Array(vertexCount);
                    for (i = 0; i < vertexCount; ++i) {
                        vertices[i] = b2Math.MulX(xf, localVertices[i]);
                    }
                    this.m_debugDraw.DrawSolidPolygon(vertices, vertexCount, color);
                }
                break;
              case b2Shape.e_edgeShape:
                {
                    var edge = shape;
                    this.m_debugDraw.DrawSegment(b2Math.MulX(xf, edge.GetVertex1()), b2Math.MulX(xf, edge.GetVertex2()), color);
                }
                break;
            }
        };
        b2World.prototype.SetDestructionListener = function(listener) {
            this.m_destructionListener = listener;
        };
        b2World.prototype.SetContactFilter = function(filter) {
            this.m_contactManager.m_contactFilter = filter;
        };
        b2World.prototype.SetContactListener = function(listener) {
            this.m_contactManager.m_contactListener = listener;
        };
        b2World.prototype.SetDebugDraw = function(debugDraw) {
            this.m_debugDraw = debugDraw;
        };
        b2World.prototype.SetBroadPhase = function(broadPhase) {
            var oldBroadPhase = this.m_contactManager.m_broadPhase;
            this.m_contactManager.m_broadPhase = broadPhase;
            for (var b = this.m_bodyList; b; b = b.m_next) {
                for (var f = b.m_fixtureList; f; f = f.m_next) {
                    f.m_proxy = broadPhase.CreateProxy(oldBroadPhase.GetFatAABB(f.m_proxy), f);
                }
            }
        };
        b2World.prototype.Validate = function() {
            this.m_contactManager.m_broadPhase.Validate();
        };
        b2World.prototype.GetProxyCount = function() {
            return this.m_contactManager.m_broadPhase.GetProxyCount();
        };
        b2World.prototype.CreateBody = function(def) {
            if (this.IsLocked() == true) {
                return null;
            }
            var b = new b2Body(def, this);
            b.m_prev = null;
            b.m_next = this.m_bodyList;
            if (this.m_bodyList) {
                this.m_bodyList.m_prev = b;
            }
            this.m_bodyList = b;
            ++this.m_bodyCount;
            return b;
        };
        b2World.prototype.DestroyBody = function(b) {
            if (this.IsLocked() == true) {
                return;
            }
            var jn = b.m_jointList;
            while (jn) {
                var jn0 = jn;
                jn = jn.next;
                if (this.m_destructionListener) {
                    this.m_destructionListener.SayGoodbyeJoint(jn0.joint);
                }
                this.DestroyJoint(jn0.joint);
            }
            var coe = b.m_controllerList;
            while (coe) {
                var coe0 = coe;
                coe = coe.nextController;
                coe0.controller.RemoveBody(b);
            }
            var ce = b.m_contactList;
            while (ce) {
                var ce0 = ce;
                ce = ce.next;
                this.m_contactManager.Destroy(ce0.contact);
            }
            b.m_contactList = null;
            var f = b.m_fixtureList;
            while (f) {
                var f0 = f;
                f = f.m_next;
                if (this.m_destructionListener) {
                    this.m_destructionListener.SayGoodbyeFixture(f0);
                }
                f0.DestroyProxy(this.m_contactManager.m_broadPhase);
                f0.Destroy();
            }
            b.m_fixtureList = null;
            b.m_fixtureCount = 0;
            if (b.m_prev) {
                b.m_prev.m_next = b.m_next;
            }
            if (b.m_next) {
                b.m_next.m_prev = b.m_prev;
            }
            if (b == this.m_bodyList) {
                this.m_bodyList = b.m_next;
            }
            --this.m_bodyCount;
        };
        b2World.prototype.CreateJoint = function(def) {
            var j = b2Joint.Create(def, null);
            j.m_prev = null;
            j.m_next = this.m_jointList;
            if (this.m_jointList) {
                this.m_jointList.m_prev = j;
            }
            this.m_jointList = j;
            ++this.m_jointCount;
            j.m_edgeA.joint = j;
            j.m_edgeA.other = j.m_bodyB;
            j.m_edgeA.prev = null;
            j.m_edgeA.next = j.m_bodyA.m_jointList;
            if (j.m_bodyA.m_jointList) j.m_bodyA.m_jointList.prev = j.m_edgeA;
            j.m_bodyA.m_jointList = j.m_edgeA;
            j.m_edgeB.joint = j;
            j.m_edgeB.other = j.m_bodyA;
            j.m_edgeB.prev = null;
            j.m_edgeB.next = j.m_bodyB.m_jointList;
            if (j.m_bodyB.m_jointList) j.m_bodyB.m_jointList.prev = j.m_edgeB;
            j.m_bodyB.m_jointList = j.m_edgeB;
            var bodyA = def.bodyA;
            var bodyB = def.bodyB;
            if (def.collideConnected == false) {
                var edge = bodyB.GetContactList();
                while (edge) {
                    if (edge.other == bodyA) {
                        edge.contact.FlagForFiltering();
                    }
                    edge = edge.next;
                }
            }
            return j;
        };
        b2World.prototype.DestroyJoint = function(j) {
            var collideConnected = j.m_collideConnected;
            if (j.m_prev) {
                j.m_prev.m_next = j.m_next;
            }
            if (j.m_next) {
                j.m_next.m_prev = j.m_prev;
            }
            if (j == this.m_jointList) {
                this.m_jointList = j.m_next;
            }
            var bodyA = j.m_bodyA;
            var bodyB = j.m_bodyB;
            bodyA.SetAwake(true);
            bodyB.SetAwake(true);
            if (j.m_edgeA.prev) {
                j.m_edgeA.prev.next = j.m_edgeA.next;
            }
            if (j.m_edgeA.next) {
                j.m_edgeA.next.prev = j.m_edgeA.prev;
            }
            if (j.m_edgeA == bodyA.m_jointList) {
                bodyA.m_jointList = j.m_edgeA.next;
            }
            j.m_edgeA.prev = null;
            j.m_edgeA.next = null;
            if (j.m_edgeB.prev) {
                j.m_edgeB.prev.next = j.m_edgeB.next;
            }
            if (j.m_edgeB.next) {
                j.m_edgeB.next.prev = j.m_edgeB.prev;
            }
            if (j.m_edgeB == bodyB.m_jointList) {
                bodyB.m_jointList = j.m_edgeB.next;
            }
            j.m_edgeB.prev = null;
            j.m_edgeB.next = null;
            b2Joint.Destroy(j, null);
            --this.m_jointCount;
            if (collideConnected == false) {
                var edge = bodyB.GetContactList();
                while (edge) {
                    if (edge.other == bodyA) {
                        edge.contact.FlagForFiltering();
                    }
                    edge = edge.next;
                }
            }
        };
        b2World.prototype.AddController = function(c) {
            c.m_next = this.m_controllerList;
            c.m_prev = null;
            this.m_controllerList = c;
            c.m_world = this;
            this.m_controllerCount++;
            return c;
        };
        b2World.prototype.RemoveController = function(c) {
            if (c.m_prev) c.m_prev.m_next = c.m_next;
            if (c.m_next) c.m_next.m_prev = c.m_prev;
            if (this.m_controllerList == c) this.m_controllerList = c.m_next;
            this.m_controllerCount--;
        };
        b2World.prototype.CreateController = function(controller) {
            if (controller.m_world != this) throw new Error("Controller can only be a member of one world");
            controller.m_next = this.m_controllerList;
            controller.m_prev = null;
            if (this.m_controllerList) this.m_controllerList.m_prev = controller;
            this.m_controllerList = controller;
            ++this.m_controllerCount;
            controller.m_world = this;
            return controller;
        };
        b2World.prototype.DestroyController = function(controller) {
            controller.Clear();
            if (controller.m_next) controller.m_next.m_prev = controller.m_prev;
            if (controller.m_prev) controller.m_prev.m_next = controller.m_next;
            if (controller == this.m_controllerList) this.m_controllerList = controller.m_next;
            --this.m_controllerCount;
        };
        b2World.prototype.SetWarmStarting = function(flag) {
            b2World.m_warmStarting = flag;
        };
        b2World.prototype.SetContinuousPhysics = function(flag) {
            b2World.m_continuousPhysics = flag;
        };
        b2World.prototype.GetBodyCount = function() {
            return this.m_bodyCount;
        };
        b2World.prototype.GetJointCount = function() {
            return this.m_jointCount;
        };
        b2World.prototype.GetContactCount = function() {
            return this.m_contactCount;
        };
        b2World.prototype.SetGravity = function(gravity) {
            this.m_gravity = gravity;
        };
        b2World.prototype.GetGravity = function() {
            return this.m_gravity;
        };
        b2World.prototype.GetGroundBody = function() {
            return this.m_groundBody;
        };
        b2World.prototype.Step = function(dt, velocityIterations, positionIterations) {
            if (this.m_flags & b2World.e_newFixture) {
                this.m_contactManager.FindNewContacts();
                this.m_flags &= ~b2World.e_newFixture;
            }
            this.m_flags |= b2World.e_locked;
            var step = b2World.s_timestep2;
            step.dt = dt;
            step.velocityIterations = velocityIterations;
            step.positionIterations = positionIterations;
            if (dt > 0) {
                step.inv_dt = 1 / dt;
            } else {
                step.inv_dt = 0;
            }
            step.dtRatio = this.m_inv_dt0 * dt;
            step.warmStarting = b2World.m_warmStarting;
            this.m_contactManager.Collide();
            if (step.dt > 0) {
                this.Solve(step);
            }
            if (b2World.m_continuousPhysics && step.dt > 0) {
                this.SolveTOI(step);
            }
            if (step.dt > 0) {
                this.m_inv_dt0 = step.inv_dt;
            }
            this.m_flags &= ~b2World.e_locked;
        };
        b2World.prototype.ClearForces = function() {
            for (var body = this.m_bodyList; body; body = body.m_next) {
                body.m_force.SetZero();
                body.m_torque = 0;
            }
        };
        b2World.prototype.DrawDebugData = function() {
            if (this.m_debugDraw == null) {
                return;
            }
            this.m_debugDraw.Clear();
            var flags = this.m_debugDraw.GetFlags();
            var i = 0;
            var b;
            var f;
            var s;
            var j;
            var bp;
            var invQ = new b2Vec2;
            var x1 = new b2Vec2;
            var x2 = new b2Vec2;
            var xf;
            var b1 = new b2AABB;
            var b2 = new b2AABB;
            var vs = [ new b2Vec2, new b2Vec2, new b2Vec2, new b2Vec2 ];
            var color = new b2Color(0, 0, 0);
            if (flags & b2DebugDraw.e_shapeBit) {
                for (b = this.m_bodyList; b; b = b.m_next) {
                    xf = b.m_xf;
                    for (f = b.GetFixtureList(); f; f = f.m_next) {
                        s = f.GetShape();
                        if (b.IsActive() == false) {
                            color.Set(.5, .5, .3);
                            this.DrawShape(s, xf, color);
                        } else if (b.GetType() == b2Body.b2_staticBody) {
                            color.Set(.5, .9, .5);
                            this.DrawShape(s, xf, color);
                        } else if (b.GetType() == b2Body.b2_kinematicBody) {
                            color.Set(.5, .5, .9);
                            this.DrawShape(s, xf, color);
                        } else if (b.IsAwake() == false) {
                            color.Set(.6, .6, .6);
                            this.DrawShape(s, xf, color);
                        } else {
                            color.Set(.9, .7, .7);
                            this.DrawShape(s, xf, color);
                        }
                    }
                }
            }
            if (flags & b2DebugDraw.e_jointBit) {
                for (j = this.m_jointList; j; j = j.m_next) {
                    this.DrawJoint(j);
                }
            }
            if (flags & b2DebugDraw.e_controllerBit) {
                for (var c = this.m_controllerList; c; c = c.m_next) {
                    c.Draw(this.m_debugDraw);
                }
            }
            if (flags & b2DebugDraw.e_pairBit) {
                color.Set(.3, .9, .9);
                for (var contact = this.m_contactManager.m_contactList; contact; contact = contact.GetNext()) {
                    var fixtureA = contact.GetFixtureA();
                    var fixtureB = contact.GetFixtureB();
                    var cA = fixtureA.GetAABB().GetCenter();
                    var cB = fixtureB.GetAABB().GetCenter();
                    this.m_debugDraw.DrawSegment(cA, cB, color);
                }
            }
            if (flags & b2DebugDraw.e_aabbBit) {
                bp = this.m_contactManager.m_broadPhase;
                vs = [ new b2Vec2, new b2Vec2, new b2Vec2, new b2Vec2 ];
                for (b = this.m_bodyList; b; b = b.GetNext()) {
                    if (b.IsActive() == false) {
                        continue;
                    }
                    for (f = b.GetFixtureList(); f; f = f.GetNext()) {
                        var aabb = bp.GetFatAABB(f.m_proxy);
                        vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
                        vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
                        vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
                        vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);
                        this.m_debugDraw.DrawPolygon(vs, 4, color);
                    }
                }
            }
            if (flags & b2DebugDraw.e_centerOfMassBit) {
                for (b = this.m_bodyList; b; b = b.m_next) {
                    xf = b2World.s_xf;
                    xf.R = b.m_xf.R;
                    xf.position = b.GetWorldCenter();
                    this.m_debugDraw.DrawTransform(xf);
                }
            }
        };
        b2World.prototype.QueryAABB = function(callback, aabb) {
            var broadPhase = this.m_contactManager.m_broadPhase;
            function WorldQueryWrapper(proxy) {
                return callback(broadPhase.GetUserData(proxy));
            }
            broadPhase.Query(WorldQueryWrapper, aabb);
        };
        b2World.prototype.QueryShape = function(callback, shape, transform) {
            if (transform == null) {
                transform = new b2Transform;
                transform.SetIdentity();
            }
            var broadPhase = this.m_contactManager.m_broadPhase;
            function WorldQueryWrapper(proxy) {
                var fixture = broadPhase.GetUserData(proxy);
                if (b2Shape.TestOverlap(shape, transform, fixture.GetShape(), fixture.GetBody().GetTransform())) return callback(fixture);
                return true;
            }
            var aabb = new b2AABB;
            shape.ComputeAABB(aabb, transform);
            broadPhase.Query(WorldQueryWrapper, aabb);
        };
        b2World.prototype.QueryPoint = function(callback, p) {
            var broadPhase = this.m_contactManager.m_broadPhase;
            function WorldQueryWrapper(proxy) {
                var fixture = broadPhase.GetUserData(proxy);
                if (fixture.TestPoint(p)) return callback(fixture);
                return true;
            }
            var aabb = new b2AABB;
            aabb.lowerBound.Set(p.x - b2Settings.b2_linearSlop, p.y - b2Settings.b2_linearSlop);
            aabb.upperBound.Set(p.x + b2Settings.b2_linearSlop, p.y + b2Settings.b2_linearSlop);
            broadPhase.Query(WorldQueryWrapper, aabb);
        };
        b2World.prototype.RayCast = function(callback, point1, point2) {
            var broadPhase = this.m_contactManager.m_broadPhase;
            var output = new b2RayCastOutput;
            function RayCastWrapper(input, proxy) {
                var userData = broadPhase.GetUserData(proxy);
                var fixture = userData;
                var hit = fixture.RayCast(output, input);
                if (hit) {
                    var fraction = output.fraction;
                    var point = new b2Vec2((1 - fraction) * point1.x + fraction * point2.x, (1 - fraction) * point1.y + fraction * point2.y);
                    return callback(fixture, point, output.normal, fraction);
                }
                return input.maxFraction;
            }
            var input = new b2RayCastInput(point1, point2);
            broadPhase.RayCast(RayCastWrapper, input);
        };
        b2World.prototype.RayCastOne = function(point1, point2) {
            var result;
            function RayCastOneWrapper(fixture, point, normal, fraction) {
                result = fixture;
                return fraction;
            }
            this.RayCast(RayCastOneWrapper, point1, point2);
            return result;
        };
        b2World.prototype.RayCastAll = function(point1, point2) {
            var result = new Array;
            function RayCastAllWrapper(fixture, point, normal, fraction) {
                result[result.length] = fixture;
                return 1;
            }
            this.RayCast(RayCastAllWrapper, point1, point2);
            return result;
        };
        b2World.prototype.GetBodyList = function() {
            return this.m_bodyList;
        };
        b2World.prototype.GetJointList = function() {
            return this.m_jointList;
        };
        b2World.prototype.GetContactList = function() {
            return this.m_contactList;
        };
        b2World.prototype.IsLocked = function() {
            return (this.m_flags & b2World.e_locked) > 0;
        };
        b2World.prototype.s_stack = new Array;
        b2World.prototype.m_flags = 0;
        b2World.prototype.m_contactManager = new b2ContactManager;
        b2World.prototype.m_contactSolver = new b2ContactSolver;
        b2World.prototype.m_island = new b2Island;
        b2World.prototype.m_bodyList = null;
        b2World.prototype.m_jointList = null;
        b2World.prototype.m_contactList = null;
        b2World.prototype.m_bodyCount = 0;
        b2World.prototype.m_contactCount = 0;
        b2World.prototype.m_jointCount = 0;
        b2World.prototype.m_controllerList = null;
        b2World.prototype.m_controllerCount = 0;
        b2World.prototype.m_gravity = null;
        b2World.prototype.m_allowSleep = null;
        b2World.prototype.m_groundBody = null;
        b2World.prototype.m_destructionListener = null;
        b2World.prototype.m_debugDraw = null;
        b2World.prototype.m_inv_dt0 = null;
        module.exports = b2World;
    },
    "7": function(require, module, exports, global) {
        var b2MassData = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2MassData.prototype.__constructor = function() {};
        b2MassData.prototype.__varz = function() {
            this.center = new b2Vec2(0, 0);
        };
        b2MassData.prototype.mass = 0;
        b2MassData.prototype.center = new b2Vec2(0, 0);
        b2MassData.prototype.I = 0;
        module.exports = b2MassData;
    },
    "8": function(require, module, exports, global) {
        var b2PolygonShape = function() {
            b2Shape.prototype.__varz.call(this);
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        extend(b2PolygonShape.prototype, b2Shape.prototype);
        b2PolygonShape.prototype._super = b2Shape.prototype;
        b2PolygonShape.prototype.__constructor = function() {
            this._super.__constructor.apply(this, arguments);
            this.m_type = b2Shape.e_polygonShape;
            this.m_centroid = new b2Vec2;
            this.m_vertices = new Array;
            this.m_normals = new Array;
        };
        b2PolygonShape.prototype.__varz = function() {};
        b2PolygonShape.AsArray = function(vertices, vertexCount) {
            var polygonShape = new b2PolygonShape;
            polygonShape.SetAsArray(vertices, vertexCount);
            return polygonShape;
        };
        b2PolygonShape.AsVector = function(vertices, vertexCount) {
            var polygonShape = new b2PolygonShape;
            polygonShape.SetAsVector(vertices, vertexCount);
            return polygonShape;
        };
        b2PolygonShape.AsBox = function(hx, hy) {
            var polygonShape = new b2PolygonShape;
            polygonShape.SetAsBox(hx, hy);
            return polygonShape;
        };
        b2PolygonShape.AsOrientedBox = function(hx, hy, center, angle) {
            var polygonShape = new b2PolygonShape;
            polygonShape.SetAsOrientedBox(hx, hy, center, angle);
            return polygonShape;
        };
        b2PolygonShape.AsEdge = function(v1, v2) {
            var polygonShape = new b2PolygonShape;
            polygonShape.SetAsEdge(v1, v2);
            return polygonShape;
        };
        b2PolygonShape.ComputeCentroid = function(vs, count) {
            var c = new b2Vec2;
            var area = 0;
            var p1X = 0;
            var p1Y = 0;
            var inv3 = 1 / 3;
            for (var i = 0; i < count; ++i) {
                var p2 = vs[i];
                var p3 = i + 1 < count ? vs[parseInt(i + 1)] : vs[0];
                var e1X = p2.x - p1X;
                var e1Y = p2.y - p1Y;
                var e2X = p3.x - p1X;
                var e2Y = p3.y - p1Y;
                var D = e1X * e2Y - e1Y * e2X;
                var triangleArea = .5 * D;
                area += triangleArea;
                c.x += triangleArea * inv3 * (p1X + p2.x + p3.x);
                c.y += triangleArea * inv3 * (p1Y + p2.y + p3.y);
            }
            c.x *= 1 / area;
            c.y *= 1 / area;
            return c;
        };
        b2PolygonShape.ComputeOBB = function(obb, vs, count) {
            var i = 0;
            var p = new Array(count + 1);
            for (i = 0; i < count; ++i) {
                p[i] = vs[i];
            }
            p[count] = p[0];
            var minArea = Number.MAX_VALUE;
            for (i = 1; i <= count; ++i) {
                var root = p[parseInt(i - 1)];
                var uxX = p[i].x - root.x;
                var uxY = p[i].y - root.y;
                var length = Math.sqrt(uxX * uxX + uxY * uxY);
                uxX /= length;
                uxY /= length;
                var uyX = -uxY;
                var uyY = uxX;
                var lowerX = Number.MAX_VALUE;
                var lowerY = Number.MAX_VALUE;
                var upperX = -Number.MAX_VALUE;
                var upperY = -Number.MAX_VALUE;
                for (var j = 0; j < count; ++j) {
                    var dX = p[j].x - root.x;
                    var dY = p[j].y - root.y;
                    var rX = uxX * dX + uxY * dY;
                    var rY = uyX * dX + uyY * dY;
                    if (rX < lowerX) lowerX = rX;
                    if (rY < lowerY) lowerY = rY;
                    if (rX > upperX) upperX = rX;
                    if (rY > upperY) upperY = rY;
                }
                var area = (upperX - lowerX) * (upperY - lowerY);
                if (area < .95 * minArea) {
                    minArea = area;
                    obb.R.col1.x = uxX;
                    obb.R.col1.y = uxY;
                    obb.R.col2.x = uyX;
                    obb.R.col2.y = uyY;
                    var centerX = .5 * (lowerX + upperX);
                    var centerY = .5 * (lowerY + upperY);
                    var tMat = obb.R;
                    obb.center.x = root.x + (tMat.col1.x * centerX + tMat.col2.x * centerY);
                    obb.center.y = root.y + (tMat.col1.y * centerX + tMat.col2.y * centerY);
                    obb.extents.x = .5 * (upperX - lowerX);
                    obb.extents.y = .5 * (upperY - lowerY);
                }
            }
        };
        b2PolygonShape.s_mat = new b2Mat22;
        b2PolygonShape.prototype.Validate = function() {
            return false;
        };
        b2PolygonShape.prototype.Reserve = function(count) {
            for (var i = this.m_vertices.length; i < count; i++) {
                this.m_vertices[i] = new b2Vec2;
                this.m_normals[i] = new b2Vec2;
            }
        };
        b2PolygonShape.prototype.Copy = function() {
            var s = new b2PolygonShape;
            s.Set(this);
            return s;
        };
        b2PolygonShape.prototype.Set = function(other) {
            this._super.Set.apply(this, [ other ]);
            if (isInstanceOf(other, b2PolygonShape)) {
                var other2 = other;
                this.m_centroid.SetV(other2.m_centroid);
                this.m_vertexCount = other2.m_vertexCount;
                this.Reserve(this.m_vertexCount);
                for (var i = 0; i < this.m_vertexCount; i++) {
                    this.m_vertices[i].SetV(other2.m_vertices[i]);
                    this.m_normals[i].SetV(other2.m_normals[i]);
                }
            }
        };
        b2PolygonShape.prototype.SetAsArray = function(vertices, vertexCount) {
            var v = new Array;
            for (var i = 0, tVec = null; i < vertices.length, tVec = vertices[i]; i++) {
                v.push(tVec);
            }
            this.SetAsVector(v, vertexCount);
        };
        b2PolygonShape.prototype.SetAsVector = function(vertices, vertexCount) {
            if (typeof vertexCount == "undefined") vertexCount = vertices.length;
            b2Settings.b2Assert(2 <= vertexCount);
            this.m_vertexCount = vertexCount;
            this.Reserve(vertexCount);
            var i = 0;
            for (i = 0; i < this.m_vertexCount; i++) {
                this.m_vertices[i].SetV(vertices[i]);
            }
            for (i = 0; i < this.m_vertexCount; ++i) {
                var i1 = i;
                var i2 = i + 1 < this.m_vertexCount ? i + 1 : 0;
                var edge = b2Math.SubtractVV(this.m_vertices[i2], this.m_vertices[i1]);
                b2Settings.b2Assert(edge.LengthSquared() > Number.MIN_VALUE);
                this.m_normals[i].SetV(b2Math.CrossVF(edge, 1));
                this.m_normals[i].Normalize();
            }
            this.m_centroid = b2PolygonShape.ComputeCentroid(this.m_vertices, this.m_vertexCount);
        };
        b2PolygonShape.prototype.SetAsBox = function(hx, hy) {
            this.m_vertexCount = 4;
            this.Reserve(4);
            this.m_vertices[0].Set(-hx, -hy);
            this.m_vertices[1].Set(hx, -hy);
            this.m_vertices[2].Set(hx, hy);
            this.m_vertices[3].Set(-hx, hy);
            this.m_normals[0].Set(0, -1);
            this.m_normals[1].Set(1, 0);
            this.m_normals[2].Set(0, 1);
            this.m_normals[3].Set(-1, 0);
            this.m_centroid.SetZero();
        };
        b2PolygonShape.prototype.SetAsOrientedBox = function(hx, hy, center, angle) {
            this.m_vertexCount = 4;
            this.Reserve(4);
            this.m_vertices[0].Set(-hx, -hy);
            this.m_vertices[1].Set(hx, -hy);
            this.m_vertices[2].Set(hx, hy);
            this.m_vertices[3].Set(-hx, hy);
            this.m_normals[0].Set(0, -1);
            this.m_normals[1].Set(1, 0);
            this.m_normals[2].Set(0, 1);
            this.m_normals[3].Set(-1, 0);
            this.m_centroid = center;
            var xf = new b2Transform;
            xf.position = center;
            xf.R.Set(angle);
            for (var i = 0; i < this.m_vertexCount; ++i) {
                this.m_vertices[i] = b2Math.MulX(xf, this.m_vertices[i]);
                this.m_normals[i] = b2Math.MulMV(xf.R, this.m_normals[i]);
            }
        };
        b2PolygonShape.prototype.SetAsEdge = function(v1, v2) {
            this.m_vertexCount = 2;
            this.Reserve(2);
            this.m_vertices[0].SetV(v1);
            this.m_vertices[1].SetV(v2);
            this.m_centroid.x = .5 * (v1.x + v2.x);
            this.m_centroid.y = .5 * (v1.y + v2.y);
            this.m_normals[0] = b2Math.CrossVF(b2Math.SubtractVV(v2, v1), 1);
            this.m_normals[0].Normalize();
            this.m_normals[1].x = -this.m_normals[0].x;
            this.m_normals[1].y = -this.m_normals[0].y;
        };
        b2PolygonShape.prototype.TestPoint = function(xf, p) {
            var tVec;
            var tMat = xf.R;
            var tX = p.x - xf.position.x;
            var tY = p.y - xf.position.y;
            var pLocalX = tX * tMat.col1.x + tY * tMat.col1.y;
            var pLocalY = tX * tMat.col2.x + tY * tMat.col2.y;
            for (var i = 0; i < this.m_vertexCount; ++i) {
                tVec = this.m_vertices[i];
                tX = pLocalX - tVec.x;
                tY = pLocalY - tVec.y;
                tVec = this.m_normals[i];
                var dot = tVec.x * tX + tVec.y * tY;
                if (dot > 0) {
                    return false;
                }
            }
            return true;
        };
        b2PolygonShape.prototype.RayCast = function(output, input, transform) {
            var lower = 0;
            var upper = input.maxFraction;
            var tX;
            var tY;
            var tMat;
            var tVec;
            tX = input.p1.x - transform.position.x;
            tY = input.p1.y - transform.position.y;
            tMat = transform.R;
            var p1X = tX * tMat.col1.x + tY * tMat.col1.y;
            var p1Y = tX * tMat.col2.x + tY * tMat.col2.y;
            tX = input.p2.x - transform.position.x;
            tY = input.p2.y - transform.position.y;
            tMat = transform.R;
            var p2X = tX * tMat.col1.x + tY * tMat.col1.y;
            var p2Y = tX * tMat.col2.x + tY * tMat.col2.y;
            var dX = p2X - p1X;
            var dY = p2Y - p1Y;
            var index = -1;
            for (var i = 0; i < this.m_vertexCount; ++i) {
                tVec = this.m_vertices[i];
                tX = tVec.x - p1X;
                tY = tVec.y - p1Y;
                tVec = this.m_normals[i];
                var numerator = tVec.x * tX + tVec.y * tY;
                var denominator = tVec.x * dX + tVec.y * dY;
                if (denominator == 0) {
                    if (numerator < 0) {
                        return false;
                    }
                } else {
                    if (denominator < 0 && numerator < lower * denominator) {
                        lower = numerator / denominator;
                        index = i;
                    } else if (denominator > 0 && numerator < upper * denominator) {
                        upper = numerator / denominator;
                    }
                }
                if (upper < lower - Number.MIN_VALUE) {
                    return false;
                }
            }
            if (index >= 0) {
                output.fraction = lower;
                tMat = transform.R;
                tVec = this.m_normals[index];
                output.normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
                output.normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
                return true;
            }
            return false;
        };
        b2PolygonShape.prototype.ComputeAABB = function(aabb, xf) {
            var tMat = xf.R;
            var tVec = this.m_vertices[0];
            var lowerX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
            var lowerY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
            var upperX = lowerX;
            var upperY = lowerY;
            for (var i = 1; i < this.m_vertexCount; ++i) {
                tVec = this.m_vertices[i];
                var vX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
                var vY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
                lowerX = lowerX < vX ? lowerX : vX;
                lowerY = lowerY < vY ? lowerY : vY;
                upperX = upperX > vX ? upperX : vX;
                upperY = upperY > vY ? upperY : vY;
            }
            aabb.lowerBound.x = lowerX - this.m_radius;
            aabb.lowerBound.y = lowerY - this.m_radius;
            aabb.upperBound.x = upperX + this.m_radius;
            aabb.upperBound.y = upperY + this.m_radius;
        };
        b2PolygonShape.prototype.ComputeMass = function(massData, density) {
            if (this.m_vertexCount == 2) {
                massData.center.x = .5 * (this.m_vertices[0].x + this.m_vertices[1].x);
                massData.center.y = .5 * (this.m_vertices[0].y + this.m_vertices[1].y);
                massData.mass = 0;
                massData.I = 0;
                return;
            }
            var centerX = 0;
            var centerY = 0;
            var area = 0;
            var I = 0;
            var p1X = 0;
            var p1Y = 0;
            var k_inv3 = 1 / 3;
            for (var i = 0; i < this.m_vertexCount; ++i) {
                var p2 = this.m_vertices[i];
                var p3 = i + 1 < this.m_vertexCount ? this.m_vertices[parseInt(i + 1)] : this.m_vertices[0];
                var e1X = p2.x - p1X;
                var e1Y = p2.y - p1Y;
                var e2X = p3.x - p1X;
                var e2Y = p3.y - p1Y;
                var D = e1X * e2Y - e1Y * e2X;
                var triangleArea = .5 * D;
                area += triangleArea;
                centerX += triangleArea * k_inv3 * (p1X + p2.x + p3.x);
                centerY += triangleArea * k_inv3 * (p1Y + p2.y + p3.y);
                var px = p1X;
                var py = p1Y;
                var ex1 = e1X;
                var ey1 = e1Y;
                var ex2 = e2X;
                var ey2 = e2Y;
                var intx2 = k_inv3 * (.25 * (ex1 * ex1 + ex2 * ex1 + ex2 * ex2) + (px * ex1 + px * ex2)) + .5 * px * px;
                var inty2 = k_inv3 * (.25 * (ey1 * ey1 + ey2 * ey1 + ey2 * ey2) + (py * ey1 + py * ey2)) + .5 * py * py;
                I += D * (intx2 + inty2);
            }
            massData.mass = density * area;
            centerX *= 1 / area;
            centerY *= 1 / area;
            massData.center.Set(centerX, centerY);
            massData.I = density * I;
        };
        b2PolygonShape.prototype.ComputeSubmergedArea = function(normal, offset, xf, c) {
            var normalL = b2Math.MulTMV(xf.R, normal);
            var offsetL = offset - b2Math.Dot(normal, xf.position);
            var depths = new Array;
            var diveCount = 0;
            var intoIndex = -1;
            var outoIndex = -1;
            var lastSubmerged = false;
            var i = 0;
            for (i = 0; i < this.m_vertexCount; ++i) {
                depths[i] = b2Math.Dot(normalL, this.m_vertices[i]) - offsetL;
                var isSubmerged = depths[i] < -Number.MIN_VALUE;
                if (i > 0) {
                    if (isSubmerged) {
                        if (!lastSubmerged) {
                            intoIndex = i - 1;
                            diveCount++;
                        }
                    } else {
                        if (lastSubmerged) {
                            outoIndex = i - 1;
                            diveCount++;
                        }
                    }
                }
                lastSubmerged = isSubmerged;
            }
            switch (diveCount) {
              case 0:
                if (lastSubmerged) {
                    var md = new b2MassData;
                    this.ComputeMass(md, 1);
                    c.SetV(b2Math.MulX(xf, md.center));
                    return md.mass;
                } else {
                    return 0;
                }
                break;
              case 1:
                if (intoIndex == -1) {
                    intoIndex = this.m_vertexCount - 1;
                } else {
                    outoIndex = this.m_vertexCount - 1;
                }
                break;
            }
            var intoIndex2 = (intoIndex + 1) % this.m_vertexCount;
            var outoIndex2 = (outoIndex + 1) % this.m_vertexCount;
            var intoLamdda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex]);
            var outoLamdda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex]);
            var intoVec = new b2Vec2(this.m_vertices[intoIndex].x * (1 - intoLamdda) + this.m_vertices[intoIndex2].x * intoLamdda, this.m_vertices[intoIndex].y * (1 - intoLamdda) + this.m_vertices[intoIndex2].y * intoLamdda);
            var outoVec = new b2Vec2(this.m_vertices[outoIndex].x * (1 - outoLamdda) + this.m_vertices[outoIndex2].x * outoLamdda, this.m_vertices[outoIndex].y * (1 - outoLamdda) + this.m_vertices[outoIndex2].y * outoLamdda);
            var area = 0;
            var center = new b2Vec2;
            var p2 = this.m_vertices[intoIndex2];
            var p3;
            i = intoIndex2;
            while (i != outoIndex2) {
                i = (i + 1) % this.m_vertexCount;
                if (i == outoIndex2) p3 = outoVec; else p3 = this.m_vertices[i];
                var triangleArea = .5 * ((p2.x - intoVec.x) * (p3.y - intoVec.y) - (p2.y - intoVec.y) * (p3.x - intoVec.x));
                area += triangleArea;
                center.x += triangleArea * (intoVec.x + p2.x + p3.x) / 3;
                center.y += triangleArea * (intoVec.y + p2.y + p3.y) / 3;
                p2 = p3;
            }
            center.Multiply(1 / area);
            c.SetV(b2Math.MulX(xf, center));
            return area;
        };
        b2PolygonShape.prototype.GetVertexCount = function() {
            return this.m_vertexCount;
        };
        b2PolygonShape.prototype.GetVertices = function() {
            return this.m_vertices;
        };
        b2PolygonShape.prototype.GetNormals = function() {
            return this.m_normals;
        };
        b2PolygonShape.prototype.GetSupport = function(d) {
            var bestIndex = 0;
            var bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
            for (var i = 1; i < this.m_vertexCount; ++i) {
                var value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
                if (value > bestValue) {
                    bestIndex = i;
                    bestValue = value;
                }
            }
            return bestIndex;
        };
        b2PolygonShape.prototype.GetSupportVertex = function(d) {
            var bestIndex = 0;
            var bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
            for (var i = 1; i < this.m_vertexCount; ++i) {
                var value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
                if (value > bestValue) {
                    bestIndex = i;
                    bestValue = value;
                }
            }
            return this.m_vertices[bestIndex];
        };
        b2PolygonShape.prototype.m_centroid = null;
        b2PolygonShape.prototype.m_vertices = null;
        b2PolygonShape.prototype.m_normals = null;
        b2PolygonShape.prototype.m_vertexCount = 0;
        module.exports = b2PolygonShape;
    },
    "9": function(require, module, exports, global) {
        var b2CircleShape = function() {
            b2Shape.prototype.__varz.call(this);
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        extend(b2CircleShape.prototype, b2Shape.prototype);
        b2CircleShape.prototype._super = b2Shape.prototype;
        b2CircleShape.prototype.__constructor = function(radius) {
            this._super.__constructor.apply(this, []);
            this.m_type = b2Shape.e_circleShape;
            this.m_radius = radius;
        };
        b2CircleShape.prototype.__varz = function() {
            this.m_p = new b2Vec2;
        };
        b2CircleShape.prototype.Copy = function() {
            var s = new b2CircleShape;
            s.Set(this);
            return s;
        };
        b2CircleShape.prototype.Set = function(other) {
            this._super.Set.apply(this, [ other ]);
            if (isInstanceOf(other, b2CircleShape)) {
                var other2 = other;
                this.m_p.SetV(other2.m_p);
            }
        };
        b2CircleShape.prototype.TestPoint = function(transform, p) {
            var tMat = transform.R;
            var dX = transform.position.x + (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y);
            var dY = transform.position.y + (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y);
            dX = p.x - dX;
            dY = p.y - dY;
            return dX * dX + dY * dY <= this.m_radius * this.m_radius;
        };
        b2CircleShape.prototype.RayCast = function(output, input, transform) {
            var tMat = transform.R;
            var positionX = transform.position.x + (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y);
            var positionY = transform.position.y + (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y);
            var sX = input.p1.x - positionX;
            var sY = input.p1.y - positionY;
            var b = sX * sX + sY * sY - this.m_radius * this.m_radius;
            var rX = input.p2.x - input.p1.x;
            var rY = input.p2.y - input.p1.y;
            var c = sX * rX + sY * rY;
            var rr = rX * rX + rY * rY;
            var sigma = c * c - rr * b;
            if (sigma < 0 || rr < Number.MIN_VALUE) {
                return false;
            }
            var a = -(c + Math.sqrt(sigma));
            if (0 <= a && a <= input.maxFraction * rr) {
                a /= rr;
                output.fraction = a;
                output.normal.x = sX + a * rX;
                output.normal.y = sY + a * rY;
                output.normal.Normalize();
                return true;
            }
            return false;
        };
        b2CircleShape.prototype.ComputeAABB = function(aabb, transform) {
            var tMat = transform.R;
            var pX = transform.position.x + (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y);
            var pY = transform.position.y + (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y);
            aabb.lowerBound.Set(pX - this.m_radius, pY - this.m_radius);
            aabb.upperBound.Set(pX + this.m_radius, pY + this.m_radius);
        };
        b2CircleShape.prototype.ComputeMass = function(massData, density) {
            massData.mass = density * b2Settings.b2_pi * this.m_radius * this.m_radius;
            massData.center.SetV(this.m_p);
            massData.I = massData.mass * (.5 * this.m_radius * this.m_radius + (this.m_p.x * this.m_p.x + this.m_p.y * this.m_p.y));
        };
        b2CircleShape.prototype.ComputeSubmergedArea = function(normal, offset, xf, c) {
            var p = b2Math.MulX(xf, this.m_p);
            var l = -(b2Math.Dot(normal, p) - offset);
            if (l < -this.m_radius + Number.MIN_VALUE) {
                return 0;
            }
            if (l > this.m_radius) {
                c.SetV(p);
                return Math.PI * this.m_radius * this.m_radius;
            }
            var r2 = this.m_radius * this.m_radius;
            var l2 = l * l;
            var area = r2 * (Math.asin(l / this.m_radius) + Math.PI / 2) + l * Math.sqrt(r2 - l2);
            var com = -2 / 3 * Math.pow(r2 - l2, 1.5) / area;
            c.x = p.x + normal.x * com;
            c.y = p.y + normal.y * com;
            return area;
        };
        b2CircleShape.prototype.GetLocalPosition = function() {
            return this.m_p;
        };
        b2CircleShape.prototype.SetLocalPosition = function(position) {
            this.m_p.SetV(position);
        };
        b2CircleShape.prototype.GetRadius = function() {
            return this.m_radius;
        };
        b2CircleShape.prototype.SetRadius = function(radius) {
            this.m_radius = radius;
        };
        b2CircleShape.prototype.m_p = new b2Vec2;
        module.exports = b2CircleShape;
    },
    a: function(require, module, exports, global) {
        var b2DebugDraw = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2DebugDraw.prototype.__constructor = function() {
            this.m_drawFlags = 0;
        };
        b2DebugDraw.prototype.__varz = function() {};
        b2DebugDraw.e_shapeBit = 1;
        b2DebugDraw.e_jointBit = 2;
        b2DebugDraw.e_aabbBit = 4;
        b2DebugDraw.e_pairBit = 8;
        b2DebugDraw.e_centerOfMassBit = 16;
        b2DebugDraw.e_controllerBit = 32;
        b2DebugDraw.prototype.SetFlags = function(flags) {
            this.m_drawFlags = flags;
        };
        b2DebugDraw.prototype.GetFlags = function() {
            return this.m_drawFlags;
        };
        b2DebugDraw.prototype.AppendFlags = function(flags) {
            this.m_drawFlags |= flags;
        };
        b2DebugDraw.prototype.ClearFlags = function(flags) {
            this.m_drawFlags &= ~flags;
        };
        b2DebugDraw.prototype.SetSprite = function(sprite) {
            this.m_sprite = sprite;
        };
        b2DebugDraw.prototype.GetSprite = function() {
            return this.m_sprite;
        };
        b2DebugDraw.prototype.SetDrawScale = function(drawScale) {
            this.m_drawScale = drawScale;
        };
        b2DebugDraw.prototype.GetDrawScale = function() {
            return this.m_drawScale;
        };
        b2DebugDraw.prototype.SetLineThickness = function(lineThickness) {
            this.m_lineThickness = lineThickness;
        };
        b2DebugDraw.prototype.GetLineThickness = function() {
            return this.m_lineThickness;
        };
        b2DebugDraw.prototype.SetAlpha = function(alpha) {
            this.m_alpha = alpha;
        };
        b2DebugDraw.prototype.GetAlpha = function() {
            return this.m_alpha;
        };
        b2DebugDraw.prototype.SetFillAlpha = function(alpha) {
            this.m_fillAlpha = alpha;
        };
        b2DebugDraw.prototype.GetFillAlpha = function() {
            return this.m_fillAlpha;
        };
        b2DebugDraw.prototype.SetXFormScale = function(xformScale) {
            this.m_xformScale = xformScale;
        };
        b2DebugDraw.prototype.GetXFormScale = function() {
            return this.m_xformScale;
        };
        b2DebugDraw.prototype.Clear = function() {
            this.m_sprite.clearRect(0, 0, this.m_sprite.canvas.width, this.m_sprite.canvas.height);
        };
        b2DebugDraw.prototype.Y = function(y) {
            return this.m_sprite.canvas.height - y;
        };
        b2DebugDraw.prototype.ToWorldPoint = function(localPoint) {
            return new b2Vec2(localPoint.x / this.m_drawScale, this.Y(localPoint.y) / this.m_drawScale);
        };
        b2DebugDraw.prototype.ColorStyle = function(color, alpha) {
            return "rgba(" + color.r + ", " + color.g + ", " + color.b + ", " + alpha + ")";
        };
        b2DebugDraw.prototype.DrawPolygon = function(vertices, vertexCount, color) {
            this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
            this.m_sprite.graphics.moveTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);
            for (var i = 1; i < vertexCount; i++) {
                this.m_sprite.graphics.lineTo(vertices[i].x * this.m_drawScale, vertices[i].y * this.m_drawScale);
            }
            this.m_sprite.graphics.lineTo(vertices[0].x * this.m_drawScale, vertices[0].y * this.m_drawScale);
        };
        b2DebugDraw.prototype.DrawSolidPolygon = function(vertices, vertexCount, color) {
            this.m_sprite.strokeSyle = this.ColorStyle(color, this.m_alpha);
            this.m_sprite.lineWidth = this.m_lineThickness;
            this.m_sprite.fillStyle = this.ColorStyle(color, this.m_fillAlpha);
            this.m_sprite.beginPath();
            this.m_sprite.moveTo(vertices[0].x * this.m_drawScale, this.Y(vertices[0].y * this.m_drawScale));
            for (var i = 1; i < vertexCount; i++) {
                this.m_sprite.lineTo(vertices[i].x * this.m_drawScale, this.Y(vertices[i].y * this.m_drawScale));
            }
            this.m_sprite.lineTo(vertices[0].x * this.m_drawScale, this.Y(vertices[0].y * this.m_drawScale));
            this.m_sprite.fill();
            this.m_sprite.stroke();
            this.m_sprite.closePath();
        };
        b2DebugDraw.prototype.DrawCircle = function(center, radius, color) {
            this.m_sprite.graphics.lineStyle(this.m_lineThickness, color.color, this.m_alpha);
            this.m_sprite.graphics.drawCircle(center.x * this.m_drawScale, center.y * this.m_drawScale, radius * this.m_drawScale);
        };
        b2DebugDraw.prototype.DrawSolidCircle = function(center, radius, axis, color) {
            this.m_sprite.strokeSyle = this.ColorStyle(color, this.m_alpha);
            this.m_sprite.lineWidth = this.m_lineThickness;
            this.m_sprite.fillStyle = this.ColorStyle(color, this.m_fillAlpha);
            this.m_sprite.beginPath();
            this.m_sprite.arc(center.x * this.m_drawScale, this.Y(center.y * this.m_drawScale), radius * this.m_drawScale, 0, Math.PI * 2, true);
            this.m_sprite.fill();
            this.m_sprite.stroke();
            this.m_sprite.closePath();
        };
        b2DebugDraw.prototype.DrawSegment = function(p1, p2, color) {
            this.m_sprite.lineWidth = this.m_lineThickness;
            this.m_sprite.strokeSyle = this.ColorStyle(color, this.m_alpha);
            this.m_sprite.beginPath();
            this.m_sprite.moveTo(p1.x * this.m_drawScale, this.Y(p1.y * this.m_drawScale));
            this.m_sprite.lineTo(p2.x * this.m_drawScale, this.Y(p2.y * this.m_drawScale));
            this.m_sprite.stroke();
            this.m_sprite.closePath();
        };
        b2DebugDraw.prototype.DrawTransform = function(xf) {
            this.m_sprite.lineWidth = this.m_lineThickness;
            this.m_sprite.strokeSyle = this.ColorStyle(new b2Color(255, 0, 0), this.m_alpha);
            this.m_sprite.beginPath();
            this.m_sprite.moveTo(xf.position.x * this.m_drawScale, this.Y(xf.position.y * this.m_drawScale));
            this.m_sprite.lineTo((xf.position.x + this.m_xformScale * xf.R.col1.x) * this.m_drawScale, this.Y((xf.position.y + this.m_xformScale * xf.R.col1.y) * this.m_drawScale));
            this.m_sprite.stroke();
            this.m_sprite.closePath();
            this.m_sprite.strokeSyle = this.ColorStyle(new b2Color(0, 255, 0), this.m_alpha);
            this.m_sprite.beginPath();
            this.m_sprite.moveTo(xf.position.x * this.m_drawScale, this.Y(xf.position.y * this.m_drawScale));
            this.m_sprite.lineTo((xf.position.x + this.m_xformScale * xf.R.col2.x) * this.m_drawScale, this.Y((xf.position.y + this.m_xformScale * xf.R.col2.y) * this.m_drawScale));
            this.m_sprite.stroke();
            this.m_sprite.closePath();
        };
        b2DebugDraw.prototype.m_drawFlags = 0;
        b2DebugDraw.prototype.m_sprite = null;
        b2DebugDraw.prototype.m_drawScale = 1;
        b2DebugDraw.prototype.m_lineThickness = 1;
        b2DebugDraw.prototype.m_alpha = 1;
        b2DebugDraw.prototype.m_fillAlpha = 1;
        b2DebugDraw.prototype.m_xformScale = 1;
        module.exports = b2DebugDraw;
    }
});
