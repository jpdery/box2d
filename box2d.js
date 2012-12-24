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
        box2d.BodyDef = require("5");
        box2d.Body = require("6");
        box2d.FixtureDef = require("c");
        box2d.Fixture = require("8");
        box2d.World = require("e");
        box2d.MassData = require("b");
        box2d.PolygonShape = require("10");
        box2d.CircleShape = require("11");
        box2d.DebugDraw = require("12");
        module.exports = box2d;
    },
    "1": function(require, module, exports, global) {
        var b2Math = require("2");
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
        b2Vec2.prototype.setZero = b2Vec2.prototype.SetZero;
        b2Vec2.prototype.set = b2Vec2.prototype.Set;
        b2Vec2.prototype.setV = b2Vec2.prototype.SetV;
        b2Vec2.prototype.getNegative = b2Vec2.prototype.GetNegative;
        b2Vec2.prototype.negativeSelf = b2Vec2.prototype.NegativeSelf;
        b2Vec2.prototype.copy = b2Vec2.prototype.Copy;
        b2Vec2.prototype.add = b2Vec2.prototype.Add;
        b2Vec2.prototype.subtract = b2Vec2.prototype.Subtract;
        b2Vec2.prototype.multiply = b2Vec2.prototype.Multiply;
        b2Vec2.prototype.mulM = b2Vec2.prototype.MulM;
        b2Vec2.prototype.mulTM = b2Vec2.prototype.MulTM;
        b2Vec2.prototype.crossVF = b2Vec2.prototype.CrossVF;
        b2Vec2.prototype.crossFV = b2Vec2.prototype.CrossFV;
        b2Vec2.prototype.minV = b2Vec2.prototype.MinV;
        b2Vec2.prototype.maxV = b2Vec2.prototype.MaxV;
        b2Vec2.prototype.abs = b2Vec2.prototype.Abs;
        b2Vec2.prototype.length = b2Vec2.prototype.Length;
        b2Vec2.prototype.lengthSquared = b2Vec2.prototype.LengthSquared;
        b2Vec2.prototype.normalize = b2Vec2.prototype.Normalize;
        b2Vec2.prototype.isValid = b2Vec2.prototype.IsValid;
        module.exports = b2Vec2;
    },
    "2": function(require, module, exports, global) {
        var b2Vec2 = require("1");
        var b2Transform = require("3");
        var b2Math = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2Math.prototype.__constructor = function() {};
        b2Math.prototype.__varz = function() {};
        b2Math.IsValid = function(x) {
            return isFinite(x);
        };
        b2Math.Dot = function(a, b) {
            return a.x * b.x + a.y * b.y;
        };
        b2Math.CrossVV = function(a, b) {
            return a.x * b.y - a.y * b.x;
        };
        b2Math.CrossVF = function(a, s) {
            var v = new b2Vec2(s * a.y, -s * a.x);
            return v;
        };
        b2Math.CrossFV = function(s, a) {
            var v = new b2Vec2(-s * a.y, s * a.x);
            return v;
        };
        b2Math.MulMV = function(A, v) {
            var u = new b2Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
            return u;
        };
        b2Math.MulTMV = function(A, v) {
            var u = new b2Vec2(b2Math.Dot(v, A.col1), b2Math.Dot(v, A.col2));
            return u;
        };
        b2Math.MulX = function(T, v) {
            var a = b2Math.MulMV(T.R, v);
            a.x += T.position.x;
            a.y += T.position.y;
            return a;
        };
        b2Math.MulXT = function(T, v) {
            var a = b2Math.SubtractVV(v, T.position);
            var tX = a.x * T.R.col1.x + a.y * T.R.col1.y;
            a.y = a.x * T.R.col2.x + a.y * T.R.col2.y;
            a.x = tX;
            return a;
        };
        b2Math.AddVV = function(a, b) {
            var v = new b2Vec2(a.x + b.x, a.y + b.y);
            return v;
        };
        b2Math.SubtractVV = function(a, b) {
            var v = new b2Vec2(a.x - b.x, a.y - b.y);
            return v;
        };
        b2Math.Distance = function(a, b) {
            var cX = a.x - b.x;
            var cY = a.y - b.y;
            return Math.sqrt(cX * cX + cY * cY);
        };
        b2Math.DistanceSquared = function(a, b) {
            var cX = a.x - b.x;
            var cY = a.y - b.y;
            return cX * cX + cY * cY;
        };
        b2Math.MulFV = function(s, a) {
            var v = new b2Vec2(s * a.x, s * a.y);
            return v;
        };
        b2Math.AddMM = function(A, B) {
            var C = b2Mat22.FromVV(b2Math.AddVV(A.col1, B.col1), b2Math.AddVV(A.col2, B.col2));
            return C;
        };
        b2Math.MulMM = function(A, B) {
            var C = b2Mat22.FromVV(b2Math.MulMV(A, B.col1), b2Math.MulMV(A, B.col2));
            return C;
        };
        b2Math.MulTMM = function(A, B) {
            var c1 = new b2Vec2(b2Math.Dot(A.col1, B.col1), b2Math.Dot(A.col2, B.col1));
            var c2 = new b2Vec2(b2Math.Dot(A.col1, B.col2), b2Math.Dot(A.col2, B.col2));
            var C = b2Mat22.FromVV(c1, c2);
            return C;
        };
        b2Math.Abs = function(a) {
            return a > 0 ? a : -a;
        };
        b2Math.AbsV = function(a) {
            var b = new b2Vec2(b2Math.Abs(a.x), b2Math.Abs(a.y));
            return b;
        };
        b2Math.AbsM = function(A) {
            var B = b2Mat22.FromVV(b2Math.AbsV(A.col1), b2Math.AbsV(A.col2));
            return B;
        };
        b2Math.Min = function(a, b) {
            return a < b ? a : b;
        };
        b2Math.MinV = function(a, b) {
            var c = new b2Vec2(b2Math.Min(a.x, b.x), b2Math.Min(a.y, b.y));
            return c;
        };
        b2Math.Max = function(a, b) {
            return a > b ? a : b;
        };
        b2Math.MaxV = function(a, b) {
            var c = new b2Vec2(b2Math.Max(a.x, b.x), b2Math.Max(a.y, b.y));
            return c;
        };
        b2Math.Clamp = function(a, low, high) {
            return a < low ? low : a > high ? high : a;
        };
        b2Math.ClampV = function(a, low, high) {
            return b2Math.MaxV(low, b2Math.MinV(a, high));
        };
        b2Math.Swap = function(a, b) {
            var tmp = a[0];
            a[0] = b[0];
            b[0] = tmp;
        };
        b2Math.Random = function() {
            return Math.random() * 2 - 1;
        };
        b2Math.RandomRange = function(lo, hi) {
            var r = Math.random();
            r = (hi - lo) * r + lo;
            return r;
        };
        b2Math.NextPowerOfTwo = function(x) {
            x |= x >> 1 & 2147483647;
            x |= x >> 2 & 1073741823;
            x |= x >> 4 & 268435455;
            x |= x >> 8 & 16777215;
            x |= x >> 16 & 65535;
            return x + 1;
        };
        b2Math.IsPowerOfTwo = function(x) {
            var result = x > 0 && (x & x - 1) == 0;
            return result;
        };
        b2Math.b2Vec2_zero = new b2Vec2(0, 0);
        b2Math.b2Mat22_identity = b2Mat22.FromVV(new b2Vec2(1, 0), new b2Vec2(0, 1));
        b2Math.b2Transform_identity = new b2Transform(b2Math.b2Vec2_zero, b2Math.b2Mat22_identity);
        module.exports = b2Math;
    },
    "3": function(require, module, exports, global) {
        var b2Vec2 = require("1");
        var b2Mat22 = require("4");
        var b2Transform = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2Transform.prototype.__constructor = function(pos, r) {
            if (pos) {
                this.position.SetV(pos);
                this.R.SetM(r);
            }
        };
        b2Transform.prototype.__varz = function() {
            this.position = new b2Vec2;
            this.R = new b2Mat22;
        };
        b2Transform.prototype.Initialize = function(pos, r) {
            this.position.SetV(pos);
            this.R.SetM(r);
        };
        b2Transform.prototype.SetIdentity = function() {
            this.position.SetZero();
            this.R.SetIdentity();
        };
        b2Transform.prototype.Set = function(x) {
            this.position.SetV(x.position);
            this.R.SetM(x.R);
        };
        b2Transform.prototype.GetAngle = function() {
            return Math.atan2(this.R.col1.y, this.R.col1.x);
        };
        b2Transform.prototype.position = new b2Vec2;
        b2Transform.prototype.R = new b2Mat22;
        b2Transform.prototype.initialize = b2Transform.prototype.Initialize;
        b2Transform.prototype.setIdentity = b2Transform.prototype.SetIdentity;
        b2Transform.prototype.set = b2Transform.prototype.Set;
        b2Transform.prototype.getAngle = b2Transform.prototype.GetAngle;
        module.exports = b2Transform;
    },
    "4": function(require, module, exports, global) {
        var b2Vec2 = require("1");
        var b2Mat22 = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2Mat22.prototype.__constructor = function() {
            this.col1.x = this.col2.y = 1;
        };
        b2Mat22.prototype.__varz = function() {
            this.col1 = new b2Vec2;
            this.col2 = new b2Vec2;
        };
        b2Mat22.FromAngle = function(angle) {
            var mat = new b2Mat22;
            mat.Set(angle);
            return mat;
        };
        b2Mat22.FromVV = function(c1, c2) {
            var mat = new b2Mat22;
            mat.SetVV(c1, c2);
            return mat;
        };
        b2Mat22.prototype.Set = function(angle) {
            var c = Math.cos(angle);
            var s = Math.sin(angle);
            this.col1.x = c;
            this.col2.x = -s;
            this.col1.y = s;
            this.col2.y = c;
        };
        b2Mat22.prototype.SetVV = function(c1, c2) {
            this.col1.SetV(c1);
            this.col2.SetV(c2);
        };
        b2Mat22.prototype.Copy = function() {
            var mat = new b2Mat22;
            mat.SetM(this);
            return mat;
        };
        b2Mat22.prototype.SetM = function(m) {
            this.col1.SetV(m.col1);
            this.col2.SetV(m.col2);
        };
        b2Mat22.prototype.AddM = function(m) {
            this.col1.x += m.col1.x;
            this.col1.y += m.col1.y;
            this.col2.x += m.col2.x;
            this.col2.y += m.col2.y;
        };
        b2Mat22.prototype.SetIdentity = function() {
            this.col1.x = 1;
            this.col2.x = 0;
            this.col1.y = 0;
            this.col2.y = 1;
        };
        b2Mat22.prototype.SetZero = function() {
            this.col1.x = 0;
            this.col2.x = 0;
            this.col1.y = 0;
            this.col2.y = 0;
        };
        b2Mat22.prototype.GetAngle = function() {
            return Math.atan2(this.col1.y, this.col1.x);
        };
        b2Mat22.prototype.GetInverse = function(out) {
            var a = this.col1.x;
            var b = this.col2.x;
            var c = this.col1.y;
            var d = this.col2.y;
            var det = a * d - b * c;
            if (det != 0) {
                det = 1 / det;
            }
            out.col1.x = det * d;
            out.col2.x = -det * b;
            out.col1.y = -det * c;
            out.col2.y = det * a;
            return out;
        };
        b2Mat22.prototype.Solve = function(out, bX, bY) {
            var a11 = this.col1.x;
            var a12 = this.col2.x;
            var a21 = this.col1.y;
            var a22 = this.col2.y;
            var det = a11 * a22 - a12 * a21;
            if (det != 0) {
                det = 1 / det;
            }
            out.x = det * (a22 * bX - a12 * bY);
            out.y = det * (a11 * bY - a21 * bX);
            return out;
        };
        b2Mat22.prototype.Abs = function() {
            this.col1.Abs();
            this.col2.Abs();
        };
        b2Mat22.prototype.col1 = new b2Vec2;
        b2Mat22.prototype.col2 = new b2Vec2;
        b2Mat22.prototype.set = b2Mat22.prototype.Set;
        b2Mat22.prototype.setVV = b2Mat22.prototype.SetVV;
        b2Mat22.prototype.copy = b2Mat22.prototype.Copy;
        b2Mat22.prototype.setM = b2Mat22.prototype.SetM;
        b2Mat22.prototype.addM = b2Mat22.prototype.AddM;
        b2Mat22.prototype.setIdentity = b2Mat22.prototype.SetIdentity;
        b2Mat22.prototype.setZero = b2Mat22.prototype.SetZero;
        b2Mat22.prototype.getAngle = b2Mat22.prototype.GetAngle;
        b2Mat22.prototype.getInverse = b2Mat22.prototype.GetInverse;
        b2Mat22.prototype.solve = b2Mat22.prototype.Solve;
        b2Mat22.prototype.abs = b2Mat22.prototype.Abs;
        module.exports = b2Mat22;
    },
    "5": function(require, module, exports, global) {
        var b2Vec2 = require("1");
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
    "6": function(require, module, exports, global) {
        var b2Transform = require("3");
        var b2Sweep = require("7");
        var b2Vec2 = require("1");
        var b2Fixture = require("8");
        var b2FixtureDef = require("c");
        var b2BodyDef = require("5");
        var b2Settings = require("d");
        var b2Math = require("2");
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
        b2Body.prototype.connectEdges = b2Body.prototype.connectEdges;
        b2Body.prototype.synchronizeFixtures = b2Body.prototype.SynchronizeFixtures;
        b2Body.prototype.synchronizeTransform = b2Body.prototype.SynchronizeTransform;
        b2Body.prototype.shouldCollide = b2Body.prototype.ShouldCollide;
        b2Body.prototype.advance = b2Body.prototype.Advance;
        b2Body.prototype.createFixture = b2Body.prototype.CreateFixture;
        b2Body.prototype.createFixture2 = b2Body.prototype.CreateFixture2;
        b2Body.prototype.destroyFixture = b2Body.prototype.DestroyFixture;
        b2Body.prototype.setPositionAndAngle = b2Body.prototype.SetPositionAndAngle;
        b2Body.prototype.setTransform = b2Body.prototype.SetTransform;
        b2Body.prototype.getTransform = b2Body.prototype.GetTransform;
        b2Body.prototype.getPosition = b2Body.prototype.GetPosition;
        b2Body.prototype.setPosition = b2Body.prototype.SetPosition;
        b2Body.prototype.getAngle = b2Body.prototype.GetAngle;
        b2Body.prototype.setAngle = b2Body.prototype.SetAngle;
        b2Body.prototype.getWorldCenter = b2Body.prototype.GetWorldCenter;
        b2Body.prototype.getLocalCenter = b2Body.prototype.GetLocalCenter;
        b2Body.prototype.setLinearVelocity = b2Body.prototype.SetLinearVelocity;
        b2Body.prototype.getLinearVelocity = b2Body.prototype.GetLinearVelocity;
        b2Body.prototype.setAngularVelocity = b2Body.prototype.SetAngularVelocity;
        b2Body.prototype.getAngularVelocity = b2Body.prototype.GetAngularVelocity;
        b2Body.prototype.getDefinition = b2Body.prototype.GetDefinition;
        b2Body.prototype.applyForce = b2Body.prototype.ApplyForce;
        b2Body.prototype.applyTorque = b2Body.prototype.ApplyTorque;
        b2Body.prototype.applyImpulse = b2Body.prototype.ApplyImpulse;
        b2Body.prototype.split = b2Body.prototype.Split;
        b2Body.prototype.merge = b2Body.prototype.Merge;
        b2Body.prototype.getMass = b2Body.prototype.GetMass;
        b2Body.prototype.getInertia = b2Body.prototype.GetInertia;
        b2Body.prototype.getMassData = b2Body.prototype.GetMassData;
        b2Body.prototype.setMassData = b2Body.prototype.SetMassData;
        b2Body.prototype.resetMassData = b2Body.prototype.ResetMassData;
        b2Body.prototype.getWorldPoint = b2Body.prototype.GetWorldPoint;
        b2Body.prototype.getWorldVector = b2Body.prototype.GetWorldVector;
        b2Body.prototype.getLocalPoint = b2Body.prototype.GetLocalPoint;
        b2Body.prototype.getLocalVector = b2Body.prototype.GetLocalVector;
        b2Body.prototype.getLinearVelocityFromWorldPoint = b2Body.prototype.GetLinearVelocityFromWorldPoint;
        b2Body.prototype.getLinearVelocityFromLocalPoint = b2Body.prototype.GetLinearVelocityFromLocalPoint;
        b2Body.prototype.getLinearDamping = b2Body.prototype.GetLinearDamping;
        b2Body.prototype.setLinearDamping = b2Body.prototype.SetLinearDamping;
        b2Body.prototype.getAngularDamping = b2Body.prototype.GetAngularDamping;
        b2Body.prototype.setAngularDamping = b2Body.prototype.SetAngularDamping;
        b2Body.prototype.setType = b2Body.prototype.SetType;
        b2Body.prototype.getType = b2Body.prototype.GetType;
        b2Body.prototype.setBullet = b2Body.prototype.SetBullet;
        b2Body.prototype.isBullet = b2Body.prototype.IsBullet;
        b2Body.prototype.setSleepingAllowed = b2Body.prototype.SetSleepingAllowed;
        b2Body.prototype.setAwake = b2Body.prototype.SetAwake;
        b2Body.prototype.isAwake = b2Body.prototype.IsAwake;
        b2Body.prototype.setFixedRotation = b2Body.prototype.SetFixedRotation;
        b2Body.prototype.isFixedRotation = b2Body.prototype.IsFixedRotation;
        b2Body.prototype.setActive = b2Body.prototype.SetActive;
        b2Body.prototype.isActive = b2Body.prototype.IsActive;
        b2Body.prototype.isSleepingAllowed = b2Body.prototype.IsSleepingAllowed;
        b2Body.prototype.getFixtureList = b2Body.prototype.GetFixtureList;
        b2Body.prototype.getJointList = b2Body.prototype.GetJointList;
        b2Body.prototype.getControllerList = b2Body.prototype.GetControllerList;
        b2Body.prototype.getContactList = b2Body.prototype.GetContactList;
        b2Body.prototype.getNext = b2Body.prototype.GetNext;
        b2Body.prototype.getUserData = b2Body.prototype.GetUserData;
        b2Body.prototype.setUserData = b2Body.prototype.SetUserData;
        b2Body.prototype.getWorld = b2Body.prototype.GetWorld;
        module.exports = b2Body;
    },
    "7": function(require, module, exports, global) {
        var b2Vec2 = require("1");
        var b2Sweep = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2Sweep.prototype.__constructor = function() {};
        b2Sweep.prototype.__varz = function() {
            this.localCenter = new b2Vec2;
            this.c0 = new b2Vec2;
            this.c = new b2Vec2;
        };
        b2Sweep.prototype.Set = function(other) {
            this.localCenter.SetV(other.localCenter);
            this.c0.SetV(other.c0);
            this.c.SetV(other.c);
            this.a0 = other.a0;
            this.a = other.a;
            this.t0 = other.t0;
        };
        b2Sweep.prototype.Copy = function() {
            var copy = new b2Sweep;
            copy.localCenter.SetV(this.localCenter);
            copy.c0.SetV(this.c0);
            copy.c.SetV(this.c);
            copy.a0 = this.a0;
            copy.a = this.a;
            copy.t0 = this.t0;
            return copy;
        };
        b2Sweep.prototype.GetTransform = function(xf, alpha) {
            xf.position.x = (1 - alpha) * this.c0.x + alpha * this.c.x;
            xf.position.y = (1 - alpha) * this.c0.y + alpha * this.c.y;
            var angle = (1 - alpha) * this.a0 + alpha * this.a;
            xf.R.Set(angle);
            var tMat = xf.R;
            xf.position.x -= tMat.col1.x * this.localCenter.x + tMat.col2.x * this.localCenter.y;
            xf.position.y -= tMat.col1.y * this.localCenter.x + tMat.col2.y * this.localCenter.y;
        };
        b2Sweep.prototype.Advance = function(t) {
            if (this.t0 < t && 1 - this.t0 > Number.MIN_VALUE) {
                var alpha = (t - this.t0) / (1 - this.t0);
                this.c0.x = (1 - alpha) * this.c0.x + alpha * this.c.x;
                this.c0.y = (1 - alpha) * this.c0.y + alpha * this.c.y;
                this.a0 = (1 - alpha) * this.a0 + alpha * this.a;
                this.t0 = t;
            }
        };
        b2Sweep.prototype.localCenter = new b2Vec2;
        b2Sweep.prototype.c0 = new b2Vec2;
        b2Sweep.prototype.c = new b2Vec2;
        b2Sweep.prototype.a0 = null;
        b2Sweep.prototype.a = null;
        b2Sweep.prototype.t0 = null;
        b2Sweep.prototype.set = b2Sweep.prototype.Set;
        b2Sweep.prototype.copy = b2Sweep.prototype.Copy;
        b2Sweep.prototype.getTransform = b2Sweep.prototype.GetTransform;
        b2Sweep.prototype.advance = b2Sweep.prototype.Advance;
        module.exports = b2Sweep;
    },
    "8": function(require, module, exports, global) {
        var b2AABB = require("9");
        var b2FilterData = require("a");
        var b2MassData = require("b");
        var b2Math = require("2");
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
        b2Fixture.prototype.create = b2Fixture.prototype.Create;
        b2Fixture.prototype.destroy = b2Fixture.prototype.Destroy;
        b2Fixture.prototype.createProxy = b2Fixture.prototype.CreateProxy;
        b2Fixture.prototype.destroyProxy = b2Fixture.prototype.DestroyProxy;
        b2Fixture.prototype.synchronize = b2Fixture.prototype.Synchronize;
        b2Fixture.prototype.getType = b2Fixture.prototype.GetType;
        b2Fixture.prototype.getShape = b2Fixture.prototype.GetShape;
        b2Fixture.prototype.setSensor = b2Fixture.prototype.SetSensor;
        b2Fixture.prototype.isSensor = b2Fixture.prototype.IsSensor;
        b2Fixture.prototype.setFilterData = b2Fixture.prototype.SetFilterData;
        b2Fixture.prototype.getFilterData = b2Fixture.prototype.GetFilterData;
        b2Fixture.prototype.getBody = b2Fixture.prototype.GetBody;
        b2Fixture.prototype.getNext = b2Fixture.prototype.GetNext;
        b2Fixture.prototype.getUserData = b2Fixture.prototype.GetUserData;
        b2Fixture.prototype.setUserData = b2Fixture.prototype.SetUserData;
        b2Fixture.prototype.testPoint = b2Fixture.prototype.TestPoint;
        b2Fixture.prototype.rayCast = b2Fixture.prototype.RayCast;
        b2Fixture.prototype.getMassData = b2Fixture.prototype.GetMassData;
        b2Fixture.prototype.setDensity = b2Fixture.prototype.SetDensity;
        b2Fixture.prototype.getDensity = b2Fixture.prototype.GetDensity;
        b2Fixture.prototype.getFriction = b2Fixture.prototype.GetFriction;
        b2Fixture.prototype.setFriction = b2Fixture.prototype.SetFriction;
        b2Fixture.prototype.getRestitution = b2Fixture.prototype.GetRestitution;
        b2Fixture.prototype.setRestitution = b2Fixture.prototype.SetRestitution;
        b2Fixture.prototype.getAABB = b2Fixture.prototype.GetAABB;
        module.exports = b2Fixture;
    },
    "9": function(require, module, exports, global) {
        var b2Vec2 = require("1");
        var b2AABB = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2AABB.prototype.__constructor = function() {};
        b2AABB.prototype.__varz = function() {
            this.lowerBound = new b2Vec2;
            this.upperBound = new b2Vec2;
        };
        b2AABB.Combine = function(aabb1, aabb2) {
            var aabb = new b2AABB;
            aabb.Combine(aabb1, aabb2);
            return aabb;
        };
        b2AABB.prototype.IsValid = function() {
            var dX = this.upperBound.x - this.lowerBound.x;
            var dY = this.upperBound.y - this.lowerBound.y;
            var valid = dX >= 0 && dY >= 0;
            valid = valid && this.lowerBound.IsValid() && this.upperBound.IsValid();
            return valid;
        };
        b2AABB.prototype.GetCenter = function() {
            return new b2Vec2((this.lowerBound.x + this.upperBound.x) / 2, (this.lowerBound.y + this.upperBound.y) / 2);
        };
        b2AABB.prototype.GetExtents = function() {
            return new b2Vec2((this.upperBound.x - this.lowerBound.x) / 2, (this.upperBound.y - this.lowerBound.y) / 2);
        };
        b2AABB.prototype.Contains = function(aabb) {
            var result = true && this.lowerBound.x <= aabb.lowerBound.x && this.lowerBound.y <= aabb.lowerBound.y && aabb.upperBound.x <= this.upperBound.x && aabb.upperBound.y <= this.upperBound.y;
            return result;
        };
        b2AABB.prototype.RayCast = function(output, input) {
            var tmin = -Number.MAX_VALUE;
            var tmax = Number.MAX_VALUE;
            var pX = input.p1.x;
            var pY = input.p1.y;
            var dX = input.p2.x - input.p1.x;
            var dY = input.p2.y - input.p1.y;
            var absDX = Math.abs(dX);
            var absDY = Math.abs(dY);
            var normal = output.normal;
            var inv_d;
            var t1;
            var t2;
            var t3;
            var s;
            {
                if (absDX < Number.MIN_VALUE) {
                    if (pX < this.lowerBound.x || this.upperBound.x < pX) return false;
                } else {
                    inv_d = 1 / dX;
                    t1 = (this.lowerBound.x - pX) * inv_d;
                    t2 = (this.upperBound.x - pX) * inv_d;
                    s = -1;
                    if (t1 > t2) {
                        t3 = t1;
                        t1 = t2;
                        t2 = t3;
                        s = 1;
                    }
                    if (t1 > tmin) {
                        normal.x = s;
                        normal.y = 0;
                        tmin = t1;
                    }
                    tmax = Math.min(tmax, t2);
                    if (tmin > tmax) return false;
                }
            }
            {
                if (absDY < Number.MIN_VALUE) {
                    if (pY < this.lowerBound.y || this.upperBound.y < pY) return false;
                } else {
                    inv_d = 1 / dY;
                    t1 = (this.lowerBound.y - pY) * inv_d;
                    t2 = (this.upperBound.y - pY) * inv_d;
                    s = -1;
                    if (t1 > t2) {
                        t3 = t1;
                        t1 = t2;
                        t2 = t3;
                        s = 1;
                    }
                    if (t1 > tmin) {
                        normal.y = s;
                        normal.x = 0;
                        tmin = t1;
                    }
                    tmax = Math.min(tmax, t2);
                    if (tmin > tmax) return false;
                }
            }
            output.fraction = tmin;
            return true;
        };
        b2AABB.prototype.TestOverlap = function(other) {
            var d1X = other.lowerBound.x - this.upperBound.x;
            var d1Y = other.lowerBound.y - this.upperBound.y;
            var d2X = this.lowerBound.x - other.upperBound.x;
            var d2Y = this.lowerBound.y - other.upperBound.y;
            if (d1X > 0 || d1Y > 0) return false;
            if (d2X > 0 || d2Y > 0) return false;
            return true;
        };
        b2AABB.prototype.Combine = function(aabb1, aabb2) {
            this.lowerBound.x = Math.min(aabb1.lowerBound.x, aabb2.lowerBound.x);
            this.lowerBound.y = Math.min(aabb1.lowerBound.y, aabb2.lowerBound.y);
            this.upperBound.x = Math.max(aabb1.upperBound.x, aabb2.upperBound.x);
            this.upperBound.y = Math.max(aabb1.upperBound.y, aabb2.upperBound.y);
        };
        b2AABB.prototype.lowerBound = new b2Vec2;
        b2AABB.prototype.upperBound = new b2Vec2;
        b2AABB.prototype.isValid = b2AABB.prototype.IsValid;
        b2AABB.prototype.getCenter = b2AABB.prototype.GetCenter;
        b2AABB.prototype.getExtents = b2AABB.prototype.GetExtents;
        b2AABB.prototype.contains = b2AABB.prototype.Contains;
        b2AABB.prototype.rayCast = b2AABB.prototype.RayCast;
        b2AABB.prototype.testOverlap = b2AABB.prototype.TestOverlap;
        b2AABB.prototype.combine = b2AABB.prototype.Combine;
        module.exports = b2AABB;
    },
    a: function(require, module, exports, global) {
        var b2FilterData = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2FilterData.prototype.__constructor = function() {};
        b2FilterData.prototype.__varz = function() {
            this.categoryBits = 1;
            this.maskBits = 65535;
        };
        b2FilterData.prototype.Copy = function() {
            var copy = new b2FilterData;
            copy.categoryBits = this.categoryBits;
            copy.maskBits = this.maskBits;
            copy.groupIndex = this.groupIndex;
            return copy;
        };
        b2FilterData.prototype.categoryBits = 1;
        b2FilterData.prototype.maskBits = 65535;
        b2FilterData.prototype.groupIndex = 0;
        b2FilterData.prototype.copy = b2FilterData.prototype.Copy;
        module.exports = b2FilterData;
    },
    b: function(require, module, exports, global) {
        var b2Vec2 = require("1");
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
    c: function(require, module, exports, global) {
        var b2FilterData = require("a");
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
    d: function(require, module, exports, global) {
        var b2Settings = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2Settings.prototype.__constructor = function() {};
        b2Settings.prototype.__varz = function() {};
        b2Settings.b2MixFriction = function(friction1, friction2) {
            return Math.sqrt(friction1 * friction2);
        };
        b2Settings.b2MixRestitution = function(restitution1, restitution2) {
            return restitution1 > restitution2 ? restitution1 : restitution2;
        };
        b2Settings.b2Assert = function(a) {
            if (!a) {
                throw "Assertion Failed";
            }
        };
        b2Settings.VERSION = "2.1alpha";
        b2Settings.USHRT_MAX = 65535;
        b2Settings.b2_pi = Math.PI;
        b2Settings.b2_maxManifoldPoints = 2;
        b2Settings.b2_aabbExtension = .1;
        b2Settings.b2_aabbMultiplier = 2;
        b2Settings.b2_polygonRadius = 2 * b2Settings.b2_linearSlop;
        b2Settings.b2_linearSlop = .005;
        b2Settings.b2_angularSlop = 2 / 180 * b2Settings.b2_pi;
        b2Settings.b2_toiSlop = 8 * b2Settings.b2_linearSlop;
        b2Settings.b2_maxTOIContactsPerIsland = 32;
        b2Settings.b2_maxTOIJointsPerIsland = 32;
        b2Settings.b2_velocityThreshold = 1;
        b2Settings.b2_maxLinearCorrection = .2;
        b2Settings.b2_maxAngularCorrection = 8 / 180 * b2Settings.b2_pi;
        b2Settings.b2_maxTranslation = 2;
        b2Settings.b2_maxTranslationSquared = b2Settings.b2_maxTranslation * b2Settings.b2_maxTranslation;
        b2Settings.b2_maxRotation = .5 * b2Settings.b2_pi;
        b2Settings.b2_maxRotationSquared = b2Settings.b2_maxRotation * b2Settings.b2_maxRotation;
        b2Settings.b2_contactBaumgarte = .2;
        b2Settings.b2_timeToSleep = .5;
        b2Settings.b2_linearSleepTolerance = .01;
        b2Settings.b2_angularSleepTolerance = 2 / 180 * b2Settings.b2_pi;
        module.exports = b2Settings;
    },
    e: function(require, module, exports, global) {
        var b2BodyDef = require("5");
        var b2ContactManager = require("f");
        var b2ContactSolver = require("q");
        var b2Island = require("w");
        var b2TimeStep = require("r");
        var b2Sweep = require("7");
        var b2Transform = require("3");
        var b2Color = require("y");
        var b2Body = require("6");
        var b2Vec2 = require("1");
        var b2AABB = require("9");
        var b2RayCastOutput = require("z");
        var b2RayCastInput = require("l");
        var b2Settings = require("d");
        var b2Math = require("2");
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
        b2World.prototype.solve = b2World.prototype.Solve;
        b2World.prototype.solveTOI = b2World.prototype.SolveTOI;
        b2World.prototype.drawJoint = b2World.prototype.DrawJoint;
        b2World.prototype.drawShape = b2World.prototype.DrawShape;
        b2World.prototype.setDestructionListener = b2World.prototype.SetDestructionListener;
        b2World.prototype.setContactFilter = b2World.prototype.SetContactFilter;
        b2World.prototype.setContactListener = b2World.prototype.SetContactListener;
        b2World.prototype.setDebugDraw = b2World.prototype.SetDebugDraw;
        b2World.prototype.setBroadPhase = b2World.prototype.SetBroadPhase;
        b2World.prototype.validate = b2World.prototype.Validate;
        b2World.prototype.getProxyCount = b2World.prototype.GetProxyCount;
        b2World.prototype.createBody = b2World.prototype.CreateBody;
        b2World.prototype.destroyBody = b2World.prototype.DestroyBody;
        b2World.prototype.createJoint = b2World.prototype.CreateJoint;
        b2World.prototype.destroyJoint = b2World.prototype.DestroyJoint;
        b2World.prototype.addController = b2World.prototype.AddController;
        b2World.prototype.removeController = b2World.prototype.RemoveController;
        b2World.prototype.createController = b2World.prototype.CreateController;
        b2World.prototype.destroyController = b2World.prototype.DestroyController;
        b2World.prototype.setWarmStarting = b2World.prototype.SetWarmStarting;
        b2World.prototype.setContinuousPhysics = b2World.prototype.SetContinuousPhysics;
        b2World.prototype.getBodyCount = b2World.prototype.GetBodyCount;
        b2World.prototype.getJointCount = b2World.prototype.GetJointCount;
        b2World.prototype.getContactCount = b2World.prototype.GetContactCount;
        b2World.prototype.setGravity = b2World.prototype.SetGravity;
        b2World.prototype.getGravity = b2World.prototype.GetGravity;
        b2World.prototype.getGroundBody = b2World.prototype.GetGroundBody;
        b2World.prototype.step = b2World.prototype.Step;
        b2World.prototype.clearForces = b2World.prototype.ClearForces;
        b2World.prototype.drawDebugData = b2World.prototype.DrawDebugData;
        b2World.prototype.queryAABB = b2World.prototype.QueryAABB;
        b2World.prototype.queryShape = b2World.prototype.QueryShape;
        b2World.prototype.queryPoint = b2World.prototype.QueryPoint;
        b2World.prototype.rayCast = b2World.prototype.RayCast;
        b2World.prototype.rayCastOne = b2World.prototype.RayCastOne;
        b2World.prototype.rayCastAll = b2World.prototype.RayCastAll;
        b2World.prototype.getBodyList = b2World.prototype.GetBodyList;
        b2World.prototype.getJointList = b2World.prototype.GetJointList;
        b2World.prototype.getContactList = b2World.prototype.GetContactList;
        b2World.prototype.isLocked = b2World.prototype.IsLocked;
        module.exports = b2World;
    },
    f: function(require, module, exports, global) {
        var b2ContactFactory = require("g");
        var b2DynamicTreeBroadPhase = require("i");
        var b2ContactPoint = require("n");
        var b2ContactManager = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2ContactManager.prototype.__constructor = function() {
            this.m_world = null;
            this.m_contactCount = 0;
            this.m_contactFilter = b2ContactFilter.b2_defaultFilter;
            this.m_contactListener = b2ContactListener.b2_defaultListener;
            this.m_contactFactory = new b2ContactFactory(this.m_allocator);
            this.m_broadPhase = new b2DynamicTreeBroadPhase;
        };
        b2ContactManager.prototype.__varz = function() {};
        b2ContactManager.s_evalCP = new b2ContactPoint;
        b2ContactManager.prototype.AddPair = function(proxyUserDataA, proxyUserDataB) {
            var fixtureA = proxyUserDataA;
            var fixtureB = proxyUserDataB;
            var bodyA = fixtureA.GetBody();
            var bodyB = fixtureB.GetBody();
            if (bodyA == bodyB) return;
            var edge = bodyB.GetContactList();
            while (edge) {
                if (edge.other == bodyA) {
                    var fA = edge.contact.GetFixtureA();
                    var fB = edge.contact.GetFixtureB();
                    if (fA == fixtureA && fB == fixtureB) return;
                    if (fA == fixtureB && fB == fixtureA) return;
                }
                edge = edge.next;
            }
            if (bodyB.ShouldCollide(bodyA) == false) {
                return;
            }
            if (this.m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false) {
                return;
            }
            var c = this.m_contactFactory.Create(fixtureA, fixtureB);
            fixtureA = c.GetFixtureA();
            fixtureB = c.GetFixtureB();
            bodyA = fixtureA.m_body;
            bodyB = fixtureB.m_body;
            c.m_prev = null;
            c.m_next = this.m_world.m_contactList;
            if (this.m_world.m_contactList != null) {
                this.m_world.m_contactList.m_prev = c;
            }
            this.m_world.m_contactList = c;
            c.m_nodeA.contact = c;
            c.m_nodeA.other = bodyB;
            c.m_nodeA.prev = null;
            c.m_nodeA.next = bodyA.m_contactList;
            if (bodyA.m_contactList != null) {
                bodyA.m_contactList.prev = c.m_nodeA;
            }
            bodyA.m_contactList = c.m_nodeA;
            c.m_nodeB.contact = c;
            c.m_nodeB.other = bodyA;
            c.m_nodeB.prev = null;
            c.m_nodeB.next = bodyB.m_contactList;
            if (bodyB.m_contactList != null) {
                bodyB.m_contactList.prev = c.m_nodeB;
            }
            bodyB.m_contactList = c.m_nodeB;
            ++this.m_world.m_contactCount;
            return;
        };
        b2ContactManager.prototype.FindNewContacts = function() {
            var that = this;
            this.m_broadPhase.UpdatePairs(function(a, b) {
                return that.AddPair(a, b);
            });
        };
        b2ContactManager.prototype.Destroy = function(c) {
            var fixtureA = c.GetFixtureA();
            var fixtureB = c.GetFixtureB();
            var bodyA = fixtureA.GetBody();
            var bodyB = fixtureB.GetBody();
            if (c.IsTouching()) {
                this.m_contactListener.EndContact(c);
            }
            if (c.m_prev) {
                c.m_prev.m_next = c.m_next;
            }
            if (c.m_next) {
                c.m_next.m_prev = c.m_prev;
            }
            if (c == this.m_world.m_contactList) {
                this.m_world.m_contactList = c.m_next;
            }
            if (c.m_nodeA.prev) {
                c.m_nodeA.prev.next = c.m_nodeA.next;
            }
            if (c.m_nodeA.next) {
                c.m_nodeA.next.prev = c.m_nodeA.prev;
            }
            if (c.m_nodeA == bodyA.m_contactList) {
                bodyA.m_contactList = c.m_nodeA.next;
            }
            if (c.m_nodeB.prev) {
                c.m_nodeB.prev.next = c.m_nodeB.next;
            }
            if (c.m_nodeB.next) {
                c.m_nodeB.next.prev = c.m_nodeB.prev;
            }
            if (c.m_nodeB == bodyB.m_contactList) {
                bodyB.m_contactList = c.m_nodeB.next;
            }
            this.m_contactFactory.Destroy(c);
            --this.m_contactCount;
        };
        b2ContactManager.prototype.Collide = function() {
            var c = this.m_world.m_contactList;
            while (c) {
                var fixtureA = c.GetFixtureA();
                var fixtureB = c.GetFixtureB();
                var bodyA = fixtureA.GetBody();
                var bodyB = fixtureB.GetBody();
                if (bodyA.IsAwake() == false && bodyB.IsAwake() == false) {
                    c = c.GetNext();
                    continue;
                }
                if (c.m_flags & b2Contact.e_filterFlag) {
                    if (bodyB.ShouldCollide(bodyA) == false) {
                        var cNuke = c;
                        c = cNuke.GetNext();
                        this.Destroy(cNuke);
                        continue;
                    }
                    if (this.m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false) {
                        cNuke = c;
                        c = cNuke.GetNext();
                        this.Destroy(cNuke);
                        continue;
                    }
                    c.m_flags &= ~b2Contact.e_filterFlag;
                }
                var proxyA = fixtureA.m_proxy;
                var proxyB = fixtureB.m_proxy;
                var overlap = this.m_broadPhase.TestOverlap(proxyA, proxyB);
                if (overlap == false) {
                    cNuke = c;
                    c = cNuke.GetNext();
                    this.Destroy(cNuke);
                    continue;
                }
                c.Update(this.m_contactListener);
                c = c.GetNext();
            }
        };
        b2ContactManager.prototype.m_world = null;
        b2ContactManager.prototype.m_broadPhase = null;
        b2ContactManager.prototype.m_contactList = null;
        b2ContactManager.prototype.m_contactCount = 0;
        b2ContactManager.prototype.m_contactFilter = null;
        b2ContactManager.prototype.m_contactListener = null;
        b2ContactManager.prototype.m_contactFactory = null;
        b2ContactManager.prototype.m_allocator = null;
        b2ContactManager.prototype.addPair = b2ContactManager.prototype.AddPair;
        b2ContactManager.prototype.findNewContacts = b2ContactManager.prototype.FindNewContacts;
        b2ContactManager.prototype.destroy = b2ContactManager.prototype.Destroy;
        b2ContactManager.prototype.collide = b2ContactManager.prototype.Collide;
        module.exports = b2ContactManager;
    },
    g: function(require, module, exports, global) {
        var b2ContactRegister = require("h");
        var b2ContactFactory = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2ContactFactory.prototype.__constructor = function(allocator) {
            this.m_allocator = allocator;
            this.InitializeRegisters();
        };
        b2ContactFactory.prototype.__varz = function() {};
        b2ContactFactory.prototype.AddType = function(createFcn, destroyFcn, type1, type2) {
            this.m_registers[type1][type2].createFcn = createFcn;
            this.m_registers[type1][type2].destroyFcn = destroyFcn;
            this.m_registers[type1][type2].primary = true;
            if (type1 != type2) {
                this.m_registers[type2][type1].createFcn = createFcn;
                this.m_registers[type2][type1].destroyFcn = destroyFcn;
                this.m_registers[type2][type1].primary = false;
            }
        };
        b2ContactFactory.prototype.InitializeRegisters = function() {
            this.m_registers = new Array(b2Shape.e_shapeTypeCount);
            for (var i = 0; i < b2Shape.e_shapeTypeCount; i++) {
                this.m_registers[i] = new Array(b2Shape.e_shapeTypeCount);
                for (var j = 0; j < b2Shape.e_shapeTypeCount; j++) {
                    this.m_registers[i][j] = new b2ContactRegister;
                }
            }
            this.AddType(b2CircleContact.Create, b2CircleContact.Destroy, b2Shape.e_circleShape, b2Shape.e_circleShape);
            this.AddType(b2PolyAndCircleContact.Create, b2PolyAndCircleContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_circleShape);
            this.AddType(b2PolygonContact.Create, b2PolygonContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_polygonShape);
            this.AddType(b2EdgeAndCircleContact.Create, b2EdgeAndCircleContact.Destroy, b2Shape.e_edgeShape, b2Shape.e_circleShape);
            this.AddType(b2PolyAndEdgeContact.Create, b2PolyAndEdgeContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_edgeShape);
        };
        b2ContactFactory.prototype.Create = function(fixtureA, fixtureB) {
            var type1 = fixtureA.GetType();
            var type2 = fixtureB.GetType();
            var reg = this.m_registers[type1][type2];
            var c;
            if (reg.pool) {
                c = reg.pool;
                reg.pool = c.m_next;
                reg.poolCount--;
                c.Reset(fixtureA, fixtureB);
                return c;
            }
            var createFcn = reg.createFcn;
            if (createFcn != null) {
                if (reg.primary) {
                    c = createFcn(this.m_allocator);
                    c.Reset(fixtureA, fixtureB);
                    return c;
                } else {
                    c = createFcn(this.m_allocator);
                    c.Reset(fixtureB, fixtureA);
                    return c;
                }
            } else {
                return null;
            }
        };
        b2ContactFactory.prototype.Destroy = function(contact) {
            if (contact.m_manifold.m_pointCount > 0) {
                contact.m_fixtureA.m_body.SetAwake(true);
                contact.m_fixtureB.m_body.SetAwake(true);
            }
            var type1 = contact.m_fixtureA.GetType();
            var type2 = contact.m_fixtureB.GetType();
            var reg = this.m_registers[type1][type2];
            if (true) {
                reg.poolCount++;
                contact.m_next = reg.pool;
                reg.pool = contact;
            }
            var destroyFcn = reg.destroyFcn;
            destroyFcn(contact, this.m_allocator);
        };
        b2ContactFactory.prototype.m_registers = null;
        b2ContactFactory.prototype.m_allocator = null;
        b2ContactFactory.prototype.addType = b2ContactFactory.prototype.AddType;
        b2ContactFactory.prototype.initializeRegisters = b2ContactFactory.prototype.InitializeRegisters;
        b2ContactFactory.prototype.create = b2ContactFactory.prototype.Create;
        b2ContactFactory.prototype.destroy = b2ContactFactory.prototype.Destroy;
        module.exports = b2ContactFactory;
    },
    h: function(require, module, exports, global) {
        var b2ContactRegister = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2ContactRegister.prototype.__constructor = function() {};
        b2ContactRegister.prototype.__varz = function() {};
        b2ContactRegister.prototype.createFcn = null;
        b2ContactRegister.prototype.destroyFcn = null;
        b2ContactRegister.prototype.primary = null;
        b2ContactRegister.prototype.pool = null;
        b2ContactRegister.prototype.poolCount = 0;
        module.exports = b2ContactRegister;
    },
    i: function(require, module, exports, global) {
        var b2DynamicTree = require("j");
        var b2DynamicTreePair = require("m");
        var b2DynamicTreeBroadPhase = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2DynamicTreeBroadPhase.prototype.__constructor = function() {};
        b2DynamicTreeBroadPhase.prototype.__varz = function() {
            this.m_tree = new b2DynamicTree;
            this.m_moveBuffer = new Array;
            this.m_pairBuffer = new Array;
        };
        b2DynamicTreeBroadPhase.prototype.BufferMove = function(proxy) {
            this.m_moveBuffer[this.m_moveBuffer.length] = proxy;
        };
        b2DynamicTreeBroadPhase.prototype.UnBufferMove = function(proxy) {
            var i = this.m_moveBuffer.indexOf(proxy);
            this.m_moveBuffer.splice(i, 1);
        };
        b2DynamicTreeBroadPhase.prototype.ComparePairs = function(pair1, pair2) {
            return 0;
        };
        b2DynamicTreeBroadPhase.prototype.CreateProxy = function(aabb, userData) {
            var proxy = this.m_tree.CreateProxy(aabb, userData);
            ++this.m_proxyCount;
            this.BufferMove(proxy);
            return proxy;
        };
        b2DynamicTreeBroadPhase.prototype.DestroyProxy = function(proxy) {
            this.UnBufferMove(proxy);
            --this.m_proxyCount;
            this.m_tree.DestroyProxy(proxy);
        };
        b2DynamicTreeBroadPhase.prototype.MoveProxy = function(proxy, aabb, displacement) {
            var buffer = this.m_tree.MoveProxy(proxy, aabb, displacement);
            if (buffer) {
                this.BufferMove(proxy);
            }
        };
        b2DynamicTreeBroadPhase.prototype.TestOverlap = function(proxyA, proxyB) {
            var aabbA = this.m_tree.GetFatAABB(proxyA);
            var aabbB = this.m_tree.GetFatAABB(proxyB);
            return aabbA.TestOverlap(aabbB);
        };
        b2DynamicTreeBroadPhase.prototype.GetUserData = function(proxy) {
            return this.m_tree.GetUserData(proxy);
        };
        b2DynamicTreeBroadPhase.prototype.GetFatAABB = function(proxy) {
            return this.m_tree.GetFatAABB(proxy);
        };
        b2DynamicTreeBroadPhase.prototype.GetProxyCount = function() {
            return this.m_proxyCount;
        };
        b2DynamicTreeBroadPhase.prototype.UpdatePairs = function(callback) {
            this.m_pairCount = 0;
            for (var i = 0, queryProxy = null; i < this.m_moveBuffer.length, queryProxy = this.m_moveBuffer[i]; i++) {
                var that = this;
                function QueryCallback(proxy) {
                    if (proxy == queryProxy) return true;
                    if (that.m_pairCount == that.m_pairBuffer.length) {
                        that.m_pairBuffer[that.m_pairCount] = new b2DynamicTreePair;
                    }
                    var pair = that.m_pairBuffer[that.m_pairCount];
                    pair.proxyA = proxy < queryProxy ? proxy : queryProxy;
                    pair.proxyB = proxy >= queryProxy ? proxy : queryProxy;
                    ++that.m_pairCount;
                    return true;
                }
                var fatAABB = this.m_tree.GetFatAABB(queryProxy);
                this.m_tree.Query(QueryCallback, fatAABB);
            }
            this.m_moveBuffer.length = 0;
            for (var i = 0; i < this.m_pairCount; ) {
                var primaryPair = this.m_pairBuffer[i];
                var userDataA = this.m_tree.GetUserData(primaryPair.proxyA);
                var userDataB = this.m_tree.GetUserData(primaryPair.proxyB);
                callback(userDataA, userDataB);
                ++i;
                while (i < this.m_pairCount) {
                    var pair = this.m_pairBuffer[i];
                    if (pair.proxyA != primaryPair.proxyA || pair.proxyB != primaryPair.proxyB) {
                        break;
                    }
                    ++i;
                }
            }
        };
        b2DynamicTreeBroadPhase.prototype.Query = function(callback, aabb) {
            this.m_tree.Query(callback, aabb);
        };
        b2DynamicTreeBroadPhase.prototype.RayCast = function(callback, input) {
            this.m_tree.RayCast(callback, input);
        };
        b2DynamicTreeBroadPhase.prototype.Validate = function() {};
        b2DynamicTreeBroadPhase.prototype.Rebalance = function(iterations) {
            this.m_tree.Rebalance(iterations);
        };
        b2DynamicTreeBroadPhase.prototype.m_tree = new b2DynamicTree;
        b2DynamicTreeBroadPhase.prototype.m_proxyCount = 0;
        b2DynamicTreeBroadPhase.prototype.m_moveBuffer = new Array;
        b2DynamicTreeBroadPhase.prototype.m_pairBuffer = new Array;
        b2DynamicTreeBroadPhase.prototype.m_pairCount = 0;
        b2DynamicTreeBroadPhase.prototype.bufferMove = b2DynamicTreeBroadPhase.prototype.BufferMove;
        b2DynamicTreeBroadPhase.prototype.unBufferMove = b2DynamicTreeBroadPhase.prototype.UnBufferMove;
        b2DynamicTreeBroadPhase.prototype.comparePairs = b2DynamicTreeBroadPhase.prototype.ComparePairs;
        b2DynamicTreeBroadPhase.prototype.createProxy = b2DynamicTreeBroadPhase.prototype.CreateProxy;
        b2DynamicTreeBroadPhase.prototype.destroyProxy = b2DynamicTreeBroadPhase.prototype.DestroyProxy;
        b2DynamicTreeBroadPhase.prototype.moveProxy = b2DynamicTreeBroadPhase.prototype.MoveProxy;
        b2DynamicTreeBroadPhase.prototype.testOverlap = b2DynamicTreeBroadPhase.prototype.TestOverlap;
        b2DynamicTreeBroadPhase.prototype.getUserData = b2DynamicTreeBroadPhase.prototype.GetUserData;
        b2DynamicTreeBroadPhase.prototype.getFatAABB = b2DynamicTreeBroadPhase.prototype.GetFatAABB;
        b2DynamicTreeBroadPhase.prototype.getProxyCount = b2DynamicTreeBroadPhase.prototype.GetProxyCount;
        b2DynamicTreeBroadPhase.prototype.updatePairs = b2DynamicTreeBroadPhase.prototype.UpdatePairs;
        b2DynamicTreeBroadPhase.prototype.query = b2DynamicTreeBroadPhase.prototype.Query;
        b2DynamicTreeBroadPhase.prototype.rayCast = b2DynamicTreeBroadPhase.prototype.RayCast;
        b2DynamicTreeBroadPhase.prototype.validate = b2DynamicTreeBroadPhase.prototype.Validate;
        b2DynamicTreeBroadPhase.prototype.rebalance = b2DynamicTreeBroadPhase.prototype.Rebalance;
        module.exports = b2DynamicTreeBroadPhase;
    },
    j: function(require, module, exports, global) {
        var b2DynamicTreeNode = require("k");
        var b2AABB = require("9");
        var b2RayCastInput = require("l");
        var b2Settings = require("d");
        var b2Math = require("2");
        var b2DynamicTree = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2DynamicTree.prototype.__constructor = function() {
            this.m_root = null;
            this.m_freeList = null;
            this.m_path = 0;
            this.m_insertionCount = 0;
        };
        b2DynamicTree.prototype.__varz = function() {};
        b2DynamicTree.prototype.AllocateNode = function() {
            if (this.m_freeList) {
                var node = this.m_freeList;
                this.m_freeList = node.parent;
                node.parent = null;
                node.child1 = null;
                node.child2 = null;
                return node;
            }
            return new b2DynamicTreeNode;
        };
        b2DynamicTree.prototype.FreeNode = function(node) {
            node.parent = this.m_freeList;
            this.m_freeList = node;
        };
        b2DynamicTree.prototype.InsertLeaf = function(leaf) {
            ++this.m_insertionCount;
            if (this.m_root == null) {
                this.m_root = leaf;
                this.m_root.parent = null;
                return;
            }
            var center = leaf.aabb.GetCenter();
            var sibling = this.m_root;
            if (sibling.IsLeaf() == false) {
                do {
                    var child1 = sibling.child1;
                    var child2 = sibling.child2;
                    var norm1 = Math.abs((child1.aabb.lowerBound.x + child1.aabb.upperBound.x) / 2 - center.x) + Math.abs((child1.aabb.lowerBound.y + child1.aabb.upperBound.y) / 2 - center.y);
                    var norm2 = Math.abs((child2.aabb.lowerBound.x + child2.aabb.upperBound.x) / 2 - center.x) + Math.abs((child2.aabb.lowerBound.y + child2.aabb.upperBound.y) / 2 - center.y);
                    if (norm1 < norm2) {
                        sibling = child1;
                    } else {
                        sibling = child2;
                    }
                } while (sibling.IsLeaf() == false);
            }
            var node1 = sibling.parent;
            var node2 = this.AllocateNode();
            node2.parent = node1;
            node2.userData = null;
            node2.aabb.Combine(leaf.aabb, sibling.aabb);
            if (node1) {
                if (sibling.parent.child1 == sibling) {
                    node1.child1 = node2;
                } else {
                    node1.child2 = node2;
                }
                node2.child1 = sibling;
                node2.child2 = leaf;
                sibling.parent = node2;
                leaf.parent = node2;
                do {
                    if (node1.aabb.Contains(node2.aabb)) break;
                    node1.aabb.Combine(node1.child1.aabb, node1.child2.aabb);
                    node2 = node1;
                    node1 = node1.parent;
                } while (node1);
            } else {
                node2.child1 = sibling;
                node2.child2 = leaf;
                sibling.parent = node2;
                leaf.parent = node2;
                this.m_root = node2;
            }
        };
        b2DynamicTree.prototype.RemoveLeaf = function(leaf) {
            if (leaf == this.m_root) {
                this.m_root = null;
                return;
            }
            var node2 = leaf.parent;
            var node1 = node2.parent;
            var sibling;
            if (node2.child1 == leaf) {
                sibling = node2.child2;
            } else {
                sibling = node2.child1;
            }
            if (node1) {
                if (node1.child1 == node2) {
                    node1.child1 = sibling;
                } else {
                    node1.child2 = sibling;
                }
                sibling.parent = node1;
                this.FreeNode(node2);
                while (node1) {
                    var oldAABB = node1.aabb;
                    node1.aabb = b2AABB.Combine(node1.child1.aabb, node1.child2.aabb);
                    if (oldAABB.Contains(node1.aabb)) break;
                    node1 = node1.parent;
                }
            } else {
                this.m_root = sibling;
                sibling.parent = null;
                this.FreeNode(node2);
            }
        };
        b2DynamicTree.prototype.CreateProxy = function(aabb, userData) {
            var node = this.AllocateNode();
            var extendX = b2Settings.b2_aabbExtension;
            var extendY = b2Settings.b2_aabbExtension;
            node.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
            node.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
            node.aabb.upperBound.x = aabb.upperBound.x + extendX;
            node.aabb.upperBound.y = aabb.upperBound.y + extendY;
            node.userData = userData;
            this.InsertLeaf(node);
            return node;
        };
        b2DynamicTree.prototype.DestroyProxy = function(proxy) {
            this.RemoveLeaf(proxy);
            this.FreeNode(proxy);
        };
        b2DynamicTree.prototype.MoveProxy = function(proxy, aabb, displacement) {
            b2Settings.b2Assert(proxy.IsLeaf());
            if (proxy.aabb.Contains(aabb)) {
                return false;
            }
            this.RemoveLeaf(proxy);
            var extendX = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * (displacement.x > 0 ? displacement.x : -displacement.x);
            var extendY = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * (displacement.y > 0 ? displacement.y : -displacement.y);
            proxy.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
            proxy.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
            proxy.aabb.upperBound.x = aabb.upperBound.x + extendX;
            proxy.aabb.upperBound.y = aabb.upperBound.y + extendY;
            this.InsertLeaf(proxy);
            return true;
        };
        b2DynamicTree.prototype.Rebalance = function(iterations) {
            if (this.m_root == null) return;
            for (var i = 0; i < iterations; i++) {
                var node = this.m_root;
                var bit = 0;
                while (node.IsLeaf() == false) {
                    node = this.m_path >> bit & 1 ? node.child2 : node.child1;
                    bit = bit + 1 & 31;
                }
                ++this.m_path;
                this.RemoveLeaf(node);
                this.InsertLeaf(node);
            }
        };
        b2DynamicTree.prototype.GetFatAABB = function(proxy) {
            return proxy.aabb;
        };
        b2DynamicTree.prototype.GetUserData = function(proxy) {
            return proxy.userData;
        };
        b2DynamicTree.prototype.Query = function(callback, aabb) {
            if (this.m_root == null) return;
            var stack = new Array;
            var count = 0;
            stack[count++] = this.m_root;
            while (count > 0) {
                var node = stack[--count];
                if (node.aabb.TestOverlap(aabb)) {
                    if (node.IsLeaf()) {
                        var proceed = callback(node);
                        if (!proceed) return;
                    } else {
                        stack[count++] = node.child1;
                        stack[count++] = node.child2;
                    }
                }
            }
        };
        b2DynamicTree.prototype.RayCast = function(callback, input) {
            if (this.m_root == null) return;
            var p1 = input.p1;
            var p2 = input.p2;
            var r = b2Math.SubtractVV(p1, p2);
            r.Normalize();
            var v = b2Math.CrossFV(1, r);
            var abs_v = b2Math.AbsV(v);
            var maxFraction = input.maxFraction;
            var segmentAABB = new b2AABB;
            var tX;
            var tY;
            {
                tX = p1.x + maxFraction * (p2.x - p1.x);
                tY = p1.y + maxFraction * (p2.y - p1.y);
                segmentAABB.lowerBound.x = Math.min(p1.x, tX);
                segmentAABB.lowerBound.y = Math.min(p1.y, tY);
                segmentAABB.upperBound.x = Math.max(p1.x, tX);
                segmentAABB.upperBound.y = Math.max(p1.y, tY);
            }
            var stack = new Array;
            var count = 0;
            stack[count++] = this.m_root;
            while (count > 0) {
                var node = stack[--count];
                if (node.aabb.TestOverlap(segmentAABB) == false) {
                    continue;
                }
                var c = node.aabb.GetCenter();
                var h = node.aabb.GetExtents();
                var separation = Math.abs(v.x * (p1.x - c.x) + v.y * (p1.y - c.y)) - abs_v.x * h.x - abs_v.y * h.y;
                if (separation > 0) continue;
                if (node.IsLeaf()) {
                    var subInput = new b2RayCastInput;
                    subInput.p1 = input.p1;
                    subInput.p2 = input.p2;
                    subInput.maxFraction = input.maxFraction;
                    maxFraction = callback(subInput, node);
                    if (maxFraction == 0) return;
                    {
                        tX = p1.x + maxFraction * (p2.x - p1.x);
                        tY = p1.y + maxFraction * (p2.y - p1.y);
                        segmentAABB.lowerBound.x = Math.min(p1.x, tX);
                        segmentAABB.lowerBound.y = Math.min(p1.y, tY);
                        segmentAABB.upperBound.x = Math.max(p1.x, tX);
                        segmentAABB.upperBound.y = Math.max(p1.y, tY);
                    }
                } else {
                    stack[count++] = node.child1;
                    stack[count++] = node.child2;
                }
            }
        };
        b2DynamicTree.prototype.m_root = null;
        b2DynamicTree.prototype.m_freeList = null;
        b2DynamicTree.prototype.m_path = 0;
        b2DynamicTree.prototype.m_insertionCount = 0;
        b2DynamicTree.prototype.allocateNode = b2DynamicTree.prototype.AllocateNode;
        b2DynamicTree.prototype.freeNode = b2DynamicTree.prototype.FreeNode;
        b2DynamicTree.prototype.insertLeaf = b2DynamicTree.prototype.InsertLeaf;
        b2DynamicTree.prototype.removeLeaf = b2DynamicTree.prototype.RemoveLeaf;
        b2DynamicTree.prototype.createProxy = b2DynamicTree.prototype.CreateProxy;
        b2DynamicTree.prototype.destroyProxy = b2DynamicTree.prototype.DestroyProxy;
        b2DynamicTree.prototype.moveProxy = b2DynamicTree.prototype.MoveProxy;
        b2DynamicTree.prototype.rebalance = b2DynamicTree.prototype.Rebalance;
        b2DynamicTree.prototype.getFatAABB = b2DynamicTree.prototype.GetFatAABB;
        b2DynamicTree.prototype.getUserData = b2DynamicTree.prototype.GetUserData;
        b2DynamicTree.prototype.query = b2DynamicTree.prototype.Query;
        b2DynamicTree.prototype.rayCast = b2DynamicTree.prototype.RayCast;
        module.exports = b2DynamicTree;
    },
    k: function(require, module, exports, global) {
        var b2AABB = require("9");
        var b2DynamicTreeNode = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2DynamicTreeNode.prototype.__constructor = function() {};
        b2DynamicTreeNode.prototype.__varz = function() {
            this.aabb = new b2AABB;
        };
        b2DynamicTreeNode.prototype.IsLeaf = function() {
            return this.child1 == null;
        };
        b2DynamicTreeNode.prototype.userData = null;
        b2DynamicTreeNode.prototype.aabb = new b2AABB;
        b2DynamicTreeNode.prototype.parent = null;
        b2DynamicTreeNode.prototype.child1 = null;
        b2DynamicTreeNode.prototype.child2 = null;
        b2DynamicTreeNode.prototype.isLeaf = b2DynamicTreeNode.prototype.IsLeaf;
        module.exports = b2DynamicTreeNode;
    },
    l: function(require, module, exports, global) {
        var b2Vec2 = require("1");
        var b2RayCastInput = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2RayCastInput.prototype.__constructor = function(p1, p2, maxFraction) {
            if (p1) this.p1.SetV(p1);
            if (p2) this.p2.SetV(p2);
            if (maxFraction) this.maxFraction = maxFraction;
        };
        b2RayCastInput.prototype.__varz = function() {
            this.p1 = new b2Vec2;
            this.p2 = new b2Vec2;
        };
        b2RayCastInput.prototype.p1 = new b2Vec2;
        b2RayCastInput.prototype.p2 = new b2Vec2;
        b2RayCastInput.prototype.maxFraction = null;
        module.exports = b2RayCastInput;
    },
    m: function(require, module, exports, global) {
        var b2DynamicTreePair = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2DynamicTreePair.prototype.__constructor = function() {};
        b2DynamicTreePair.prototype.__varz = function() {};
        b2DynamicTreePair.prototype.proxyA = null;
        b2DynamicTreePair.prototype.proxyB = null;
        module.exports = b2DynamicTreePair;
    },
    n: function(require, module, exports, global) {
        var b2Vec2 = require("1");
        var b2ContactID = require("o");
        var b2ContactPoint = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2ContactPoint.prototype.__constructor = function() {};
        b2ContactPoint.prototype.__varz = function() {
            this.position = new b2Vec2;
            this.velocity = new b2Vec2;
            this.normal = new b2Vec2;
            this.id = new b2ContactID;
        };
        b2ContactPoint.prototype.shape1 = null;
        b2ContactPoint.prototype.shape2 = null;
        b2ContactPoint.prototype.position = new b2Vec2;
        b2ContactPoint.prototype.velocity = new b2Vec2;
        b2ContactPoint.prototype.normal = new b2Vec2;
        b2ContactPoint.prototype.separation = null;
        b2ContactPoint.prototype.friction = null;
        b2ContactPoint.prototype.restitution = null;
        b2ContactPoint.prototype.id = new b2ContactID;
        module.exports = b2ContactPoint;
    },
    o: function(require, module, exports, global) {
        var Features = require("p");
        var b2ContactID = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2ContactID.prototype.__constructor = function() {
            this.features._m_id = this;
        };
        b2ContactID.prototype.__varz = function() {
            this.features = new Features;
        };
        b2ContactID.prototype.Set = function(id) {
            key = id._key;
        };
        b2ContactID.prototype.Copy = function() {
            var id = new b2ContactID;
            id.key = key;
            return id;
        };
        b2ContactID.prototype.__defineSetter__("key", function() {
            return this._key;
        });
        b2ContactID.prototype.__defineSetter__("key", function(value) {
            this._key = value;
            this.features._referenceEdge = this._key & 255;
            this.features._incidentEdge = (this._key & 65280) >> 8 & 255;
            this.features._incidentVertex = (this._key & 16711680) >> 16 & 255;
            this.features._flip = (this._key & 4278190080) >> 24 & 255;
        });
        b2ContactID.prototype._key = 0;
        b2ContactID.prototype.features = new Features;
        b2ContactID.prototype.set = b2ContactID.prototype.Set;
        b2ContactID.prototype.copy = b2ContactID.prototype.Copy;
        module.exports = b2ContactID;
    },
    p: function(require, module, exports, global) {
        var Features = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        Features.prototype.__constructor = function() {};
        Features.prototype.__varz = function() {};
        Features.prototype.__defineGetter__("referenceEdge", function() {
            return this._referenceEdge;
        });
        Features.prototype.__defineSetter__("referenceEdge", function(value) {
            this._referenceEdge = value;
            this._m_id._key = this._m_id._key & 4294967040 | this._referenceEdge & 255;
        });
        Features.prototype.__defineGetter__("incidentEdge", function() {
            return this._incidentEdge;
        });
        Features.prototype.__defineSetter__("incidentEdge", function(value) {
            this._incidentEdge = value;
            this._m_id._key = this._m_id._key & 4294902015 | this._incidentEdge << 8 & 65280;
        });
        Features.prototype.__defineGetter__("incidentVertex", function() {
            return this._incidentVertex;
        });
        Features.prototype.__defineSetter__("incidentVertex", function(value) {
            this._incidentVertex = value;
            this._m_id._key = this._m_id._key & 4278255615 | this._incidentVertex << 16 & 16711680;
        });
        Features.prototype.__defineGetter__("flip", function() {
            return this._flip;
        });
        Features.prototype.__defineSetter__("flip", function(value) {
            this._flip = value;
            this._m_id._key = this._m_id._key & 16777215 | this._flip << 24 & 4278190080;
        });
        Features.prototype._referenceEdge = 0;
        Features.prototype._incidentEdge = 0;
        Features.prototype._incidentVertex = 0;
        Features.prototype._flip = 0;
        Features.prototype._m_id = null;
        module.exports = Features;
    },
    q: function(require, module, exports, global) {
        var b2TimeStep = require("r");
        var b2WorldManifold = require("s");
        var b2PositionSolverManifold = require("t");
        var b2ContactConstraint = require("u");
        var b2Settings = require("d");
        var b2Math = require("2");
        var b2ContactSolver = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2ContactSolver.prototype.__constructor = function() {};
        b2ContactSolver.prototype.__varz = function() {
            this.m_step = new b2TimeStep;
            this.m_constraints = new Array;
        };
        b2ContactSolver.s_worldManifold = new b2WorldManifold;
        b2ContactSolver.s_psm = new b2PositionSolverManifold;
        b2ContactSolver.prototype.Initialize = function(step, contacts, contactCount, allocator) {
            var contact;
            this.m_step.Set(step);
            this.m_allocator = allocator;
            var i = 0;
            var tVec;
            var tMat;
            this.m_constraintCount = contactCount;
            while (this.m_constraints.length < this.m_constraintCount) {
                this.m_constraints[this.m_constraints.length] = new b2ContactConstraint;
            }
            for (i = 0; i < contactCount; ++i) {
                contact = contacts[i];
                var fixtureA = contact.m_fixtureA;
                var fixtureB = contact.m_fixtureB;
                var shapeA = fixtureA.m_shape;
                var shapeB = fixtureB.m_shape;
                var radiusA = shapeA.m_radius;
                var radiusB = shapeB.m_radius;
                var bodyA = fixtureA.m_body;
                var bodyB = fixtureB.m_body;
                var manifold = contact.GetManifold();
                var friction = b2Settings.b2MixFriction(fixtureA.GetFriction(), fixtureB.GetFriction());
                var restitution = b2Settings.b2MixRestitution(fixtureA.GetRestitution(), fixtureB.GetRestitution());
                var vAX = bodyA.m_linearVelocity.x;
                var vAY = bodyA.m_linearVelocity.y;
                var vBX = bodyB.m_linearVelocity.x;
                var vBY = bodyB.m_linearVelocity.y;
                var wA = bodyA.m_angularVelocity;
                var wB = bodyB.m_angularVelocity;
                b2Settings.b2Assert(manifold.m_pointCount > 0);
                b2ContactSolver.s_worldManifold.Initialize(manifold, bodyA.m_xf, radiusA, bodyB.m_xf, radiusB);
                var normalX = b2ContactSolver.s_worldManifold.m_normal.x;
                var normalY = b2ContactSolver.s_worldManifold.m_normal.y;
                var cc = this.m_constraints[i];
                cc.bodyA = bodyA;
                cc.bodyB = bodyB;
                cc.manifold = manifold;
                cc.normal.x = normalX;
                cc.normal.y = normalY;
                cc.pointCount = manifold.m_pointCount;
                cc.friction = friction;
                cc.restitution = restitution;
                cc.localPlaneNormal.x = manifold.m_localPlaneNormal.x;
                cc.localPlaneNormal.y = manifold.m_localPlaneNormal.y;
                cc.localPoint.x = manifold.m_localPoint.x;
                cc.localPoint.y = manifold.m_localPoint.y;
                cc.radius = radiusA + radiusB;
                cc.type = manifold.m_type;
                for (var k = 0; k < cc.pointCount; ++k) {
                    var cp = manifold.m_points[k];
                    var ccp = cc.points[k];
                    ccp.normalImpulse = cp.m_normalImpulse;
                    ccp.tangentImpulse = cp.m_tangentImpulse;
                    ccp.localPoint.SetV(cp.m_localPoint);
                    var rAX = ccp.rA.x = b2ContactSolver.s_worldManifold.m_points[k].x - bodyA.m_sweep.c.x;
                    var rAY = ccp.rA.y = b2ContactSolver.s_worldManifold.m_points[k].y - bodyA.m_sweep.c.y;
                    var rBX = ccp.rB.x = b2ContactSolver.s_worldManifold.m_points[k].x - bodyB.m_sweep.c.x;
                    var rBY = ccp.rB.y = b2ContactSolver.s_worldManifold.m_points[k].y - bodyB.m_sweep.c.y;
                    var rnA = rAX * normalY - rAY * normalX;
                    var rnB = rBX * normalY - rBY * normalX;
                    rnA *= rnA;
                    rnB *= rnB;
                    var kNormal = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rnA + bodyB.m_invI * rnB;
                    ccp.normalMass = 1 / kNormal;
                    var kEqualized = bodyA.m_mass * bodyA.m_invMass + bodyB.m_mass * bodyB.m_invMass;
                    kEqualized += bodyA.m_mass * bodyA.m_invI * rnA + bodyB.m_mass * bodyB.m_invI * rnB;
                    ccp.equalizedMass = 1 / kEqualized;
                    var tangentX = normalY;
                    var tangentY = -normalX;
                    var rtA = rAX * tangentY - rAY * tangentX;
                    var rtB = rBX * tangentY - rBY * tangentX;
                    rtA *= rtA;
                    rtB *= rtB;
                    var kTangent = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rtA + bodyB.m_invI * rtB;
                    ccp.tangentMass = 1 / kTangent;
                    ccp.velocityBias = 0;
                    var tX = vBX + -wB * rBY - vAX - -wA * rAY;
                    var tY = vBY + wB * rBX - vAY - wA * rAX;
                    var vRel = cc.normal.x * tX + cc.normal.y * tY;
                    if (vRel < -b2Settings.b2_velocityThreshold) {
                        ccp.velocityBias += -cc.restitution * vRel;
                    }
                }
                if (cc.pointCount == 2) {
                    var ccp1 = cc.points[0];
                    var ccp2 = cc.points[1];
                    var invMassA = bodyA.m_invMass;
                    var invIA = bodyA.m_invI;
                    var invMassB = bodyB.m_invMass;
                    var invIB = bodyB.m_invI;
                    var rn1A = ccp1.rA.x * normalY - ccp1.rA.y * normalX;
                    var rn1B = ccp1.rB.x * normalY - ccp1.rB.y * normalX;
                    var rn2A = ccp2.rA.x * normalY - ccp2.rA.y * normalX;
                    var rn2B = ccp2.rB.x * normalY - ccp2.rB.y * normalX;
                    var k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
                    var k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
                    var k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;
                    var k_maxConditionNumber = 100;
                    if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
                        cc.K.col1.Set(k11, k12);
                        cc.K.col2.Set(k12, k22);
                        cc.K.GetInverse(cc.normalMass);
                    } else {
                        cc.pointCount = 1;
                    }
                }
            }
        };
        b2ContactSolver.prototype.InitVelocityConstraints = function(step) {
            var tVec;
            var tVec2;
            var tMat;
            for (var i = 0; i < this.m_constraintCount; ++i) {
                var c = this.m_constraints[i];
                var bodyA = c.bodyA;
                var bodyB = c.bodyB;
                var invMassA = bodyA.m_invMass;
                var invIA = bodyA.m_invI;
                var invMassB = bodyB.m_invMass;
                var invIB = bodyB.m_invI;
                var normalX = c.normal.x;
                var normalY = c.normal.y;
                var tangentX = normalY;
                var tangentY = -normalX;
                var tX;
                var j = 0;
                var tCount = 0;
                if (step.warmStarting) {
                    tCount = c.pointCount;
                    for (j = 0; j < tCount; ++j) {
                        var ccp = c.points[j];
                        ccp.normalImpulse *= step.dtRatio;
                        ccp.tangentImpulse *= step.dtRatio;
                        var PX = ccp.normalImpulse * normalX + ccp.tangentImpulse * tangentX;
                        var PY = ccp.normalImpulse * normalY + ccp.tangentImpulse * tangentY;
                        bodyA.m_angularVelocity -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
                        bodyA.m_linearVelocity.x -= invMassA * PX;
                        bodyA.m_linearVelocity.y -= invMassA * PY;
                        bodyB.m_angularVelocity += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
                        bodyB.m_linearVelocity.x += invMassB * PX;
                        bodyB.m_linearVelocity.y += invMassB * PY;
                    }
                } else {
                    tCount = c.pointCount;
                    for (j = 0; j < tCount; ++j) {
                        var ccp2 = c.points[j];
                        ccp2.normalImpulse = 0;
                        ccp2.tangentImpulse = 0;
                    }
                }
            }
        };
        b2ContactSolver.prototype.SolveVelocityConstraints = function() {
            var j = 0;
            var ccp;
            var rAX;
            var rAY;
            var rBX;
            var rBY;
            var dvX;
            var dvY;
            var vn;
            var vt;
            var lambda;
            var maxFriction;
            var newImpulse;
            var PX;
            var PY;
            var dX;
            var dY;
            var P1X;
            var P1Y;
            var P2X;
            var P2Y;
            var tMat;
            var tVec;
            for (var i = 0; i < this.m_constraintCount; ++i) {
                var c = this.m_constraints[i];
                var bodyA = c.bodyA;
                var bodyB = c.bodyB;
                var wA = bodyA.m_angularVelocity;
                var wB = bodyB.m_angularVelocity;
                var vA = bodyA.m_linearVelocity;
                var vB = bodyB.m_linearVelocity;
                var invMassA = bodyA.m_invMass;
                var invIA = bodyA.m_invI;
                var invMassB = bodyB.m_invMass;
                var invIB = bodyB.m_invI;
                var normalX = c.normal.x;
                var normalY = c.normal.y;
                var tangentX = normalY;
                var tangentY = -normalX;
                var friction = c.friction;
                var tX;
                for (j = 0; j < c.pointCount; j++) {
                    ccp = c.points[j];
                    dvX = vB.x - wB * ccp.rB.y - vA.x + wA * ccp.rA.y;
                    dvY = vB.y + wB * ccp.rB.x - vA.y - wA * ccp.rA.x;
                    vt = dvX * tangentX + dvY * tangentY;
                    lambda = ccp.tangentMass * -vt;
                    maxFriction = friction * ccp.normalImpulse;
                    newImpulse = b2Math.Clamp(ccp.tangentImpulse + lambda, -maxFriction, maxFriction);
                    lambda = newImpulse - ccp.tangentImpulse;
                    PX = lambda * tangentX;
                    PY = lambda * tangentY;
                    vA.x -= invMassA * PX;
                    vA.y -= invMassA * PY;
                    wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
                    vB.x += invMassB * PX;
                    vB.y += invMassB * PY;
                    wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
                    ccp.tangentImpulse = newImpulse;
                }
                var tCount = c.pointCount;
                if (c.pointCount == 1) {
                    ccp = c.points[0];
                    dvX = vB.x + -wB * ccp.rB.y - vA.x - -wA * ccp.rA.y;
                    dvY = vB.y + wB * ccp.rB.x - vA.y - wA * ccp.rA.x;
                    vn = dvX * normalX + dvY * normalY;
                    lambda = -ccp.normalMass * (vn - ccp.velocityBias);
                    newImpulse = ccp.normalImpulse + lambda;
                    newImpulse = newImpulse > 0 ? newImpulse : 0;
                    lambda = newImpulse - ccp.normalImpulse;
                    PX = lambda * normalX;
                    PY = lambda * normalY;
                    vA.x -= invMassA * PX;
                    vA.y -= invMassA * PY;
                    wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
                    vB.x += invMassB * PX;
                    vB.y += invMassB * PY;
                    wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
                    ccp.normalImpulse = newImpulse;
                } else {
                    var cp1 = c.points[0];
                    var cp2 = c.points[1];
                    var aX = cp1.normalImpulse;
                    var aY = cp2.normalImpulse;
                    var dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y;
                    var dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x;
                    var dv2X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y;
                    var dv2Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x;
                    var vn1 = dv1X * normalX + dv1Y * normalY;
                    var vn2 = dv2X * normalX + dv2Y * normalY;
                    var bX = vn1 - cp1.velocityBias;
                    var bY = vn2 - cp2.velocityBias;
                    tMat = c.K;
                    bX -= tMat.col1.x * aX + tMat.col2.x * aY;
                    bY -= tMat.col1.y * aX + tMat.col2.y * aY;
                    var k_errorTol = .001;
                    for (;;) {
                        tMat = c.normalMass;
                        var xX = -(tMat.col1.x * bX + tMat.col2.x * bY);
                        var xY = -(tMat.col1.y * bX + tMat.col2.y * bY);
                        if (xX >= 0 && xY >= 0) {
                            dX = xX - aX;
                            dY = xY - aY;
                            P1X = dX * normalX;
                            P1Y = dX * normalY;
                            P2X = dY * normalX;
                            P2Y = dY * normalY;
                            vA.x -= invMassA * (P1X + P2X);
                            vA.y -= invMassA * (P1Y + P2Y);
                            wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
                            vB.x += invMassB * (P1X + P2X);
                            vB.y += invMassB * (P1Y + P2Y);
                            wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
                            cp1.normalImpulse = xX;
                            cp2.normalImpulse = xY;
                            break;
                        }
                        xX = -cp1.normalMass * bX;
                        xY = 0;
                        vn1 = 0;
                        vn2 = c.K.col1.y * xX + bY;
                        if (xX >= 0 && vn2 >= 0) {
                            dX = xX - aX;
                            dY = xY - aY;
                            P1X = dX * normalX;
                            P1Y = dX * normalY;
                            P2X = dY * normalX;
                            P2Y = dY * normalY;
                            vA.x -= invMassA * (P1X + P2X);
                            vA.y -= invMassA * (P1Y + P2Y);
                            wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
                            vB.x += invMassB * (P1X + P2X);
                            vB.y += invMassB * (P1Y + P2Y);
                            wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
                            cp1.normalImpulse = xX;
                            cp2.normalImpulse = xY;
                            break;
                        }
                        xX = 0;
                        xY = -cp2.normalMass * bY;
                        vn1 = c.K.col2.x * xY + bX;
                        vn2 = 0;
                        if (xY >= 0 && vn1 >= 0) {
                            dX = xX - aX;
                            dY = xY - aY;
                            P1X = dX * normalX;
                            P1Y = dX * normalY;
                            P2X = dY * normalX;
                            P2Y = dY * normalY;
                            vA.x -= invMassA * (P1X + P2X);
                            vA.y -= invMassA * (P1Y + P2Y);
                            wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
                            vB.x += invMassB * (P1X + P2X);
                            vB.y += invMassB * (P1Y + P2Y);
                            wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
                            cp1.normalImpulse = xX;
                            cp2.normalImpulse = xY;
                            break;
                        }
                        xX = 0;
                        xY = 0;
                        vn1 = bX;
                        vn2 = bY;
                        if (vn1 >= 0 && vn2 >= 0) {
                            dX = xX - aX;
                            dY = xY - aY;
                            P1X = dX * normalX;
                            P1Y = dX * normalY;
                            P2X = dY * normalX;
                            P2Y = dY * normalY;
                            vA.x -= invMassA * (P1X + P2X);
                            vA.y -= invMassA * (P1Y + P2Y);
                            wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
                            vB.x += invMassB * (P1X + P2X);
                            vB.y += invMassB * (P1Y + P2Y);
                            wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
                            cp1.normalImpulse = xX;
                            cp2.normalImpulse = xY;
                            break;
                        }
                        break;
                    }
                }
                bodyA.m_angularVelocity = wA;
                bodyB.m_angularVelocity = wB;
            }
        };
        b2ContactSolver.prototype.FinalizeVelocityConstraints = function() {
            for (var i = 0; i < this.m_constraintCount; ++i) {
                var c = this.m_constraints[i];
                var m = c.manifold;
                for (var j = 0; j < c.pointCount; ++j) {
                    var point1 = m.m_points[j];
                    var point2 = c.points[j];
                    point1.m_normalImpulse = point2.normalImpulse;
                    point1.m_tangentImpulse = point2.tangentImpulse;
                }
            }
        };
        b2ContactSolver.prototype.SolvePositionConstraints = function(baumgarte) {
            var minSeparation = 0;
            for (var i = 0; i < this.m_constraintCount; i++) {
                var c = this.m_constraints[i];
                var bodyA = c.bodyA;
                var bodyB = c.bodyB;
                var invMassA = bodyA.m_mass * bodyA.m_invMass;
                var invIA = bodyA.m_mass * bodyA.m_invI;
                var invMassB = bodyB.m_mass * bodyB.m_invMass;
                var invIB = bodyB.m_mass * bodyB.m_invI;
                b2ContactSolver.s_psm.Initialize(c);
                var normal = b2ContactSolver.s_psm.m_normal;
                for (var j = 0; j < c.pointCount; j++) {
                    var ccp = c.points[j];
                    var point = b2ContactSolver.s_psm.m_points[j];
                    var separation = b2ContactSolver.s_psm.m_separations[j];
                    var rAX = point.x - bodyA.m_sweep.c.x;
                    var rAY = point.y - bodyA.m_sweep.c.y;
                    var rBX = point.x - bodyB.m_sweep.c.x;
                    var rBY = point.y - bodyB.m_sweep.c.y;
                    minSeparation = minSeparation < separation ? minSeparation : separation;
                    var C = b2Math.Clamp(baumgarte * (separation + b2Settings.b2_linearSlop), -b2Settings.b2_maxLinearCorrection, 0);
                    var impulse = -ccp.equalizedMass * C;
                    var PX = impulse * normal.x;
                    var PY = impulse * normal.y;
                    bodyA.m_sweep.c.x -= invMassA * PX;
                    bodyA.m_sweep.c.y -= invMassA * PY;
                    bodyA.m_sweep.a -= invIA * (rAX * PY - rAY * PX);
                    bodyA.SynchronizeTransform();
                    bodyB.m_sweep.c.x += invMassB * PX;
                    bodyB.m_sweep.c.y += invMassB * PY;
                    bodyB.m_sweep.a += invIB * (rBX * PY - rBY * PX);
                    bodyB.SynchronizeTransform();
                }
            }
            return minSeparation > -1.5 * b2Settings.b2_linearSlop;
        };
        b2ContactSolver.prototype.m_step = new b2TimeStep;
        b2ContactSolver.prototype.m_allocator = null;
        b2ContactSolver.prototype.m_constraints = new Array;
        b2ContactSolver.prototype.m_constraintCount = 0;
        b2ContactSolver.prototype.initialize = b2ContactSolver.prototype.Initialize;
        b2ContactSolver.prototype.initVelocityConstraints = b2ContactSolver.prototype.InitVelocityConstraints;
        b2ContactSolver.prototype.solveVelocityConstraints = b2ContactSolver.prototype.SolveVelocityConstraints;
        b2ContactSolver.prototype.finalizeVelocityConstraints = b2ContactSolver.prototype.FinalizeVelocityConstraints;
        b2ContactSolver.prototype.solvePositionConstraints = b2ContactSolver.prototype.SolvePositionConstraints;
        module.exports = b2ContactSolver;
    },
    r: function(require, module, exports, global) {
        var b2TimeStep = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2TimeStep.prototype.__constructor = function() {};
        b2TimeStep.prototype.__varz = function() {};
        b2TimeStep.prototype.Set = function(step) {
            this.dt = step.dt;
            this.inv_dt = step.inv_dt;
            this.positionIterations = step.positionIterations;
            this.velocityIterations = step.velocityIterations;
            this.warmStarting = step.warmStarting;
        };
        b2TimeStep.prototype.dt = null;
        b2TimeStep.prototype.inv_dt = null;
        b2TimeStep.prototype.dtRatio = null;
        b2TimeStep.prototype.velocityIterations = 0;
        b2TimeStep.prototype.positionIterations = 0;
        b2TimeStep.prototype.warmStarting = null;
        b2TimeStep.prototype.set = b2TimeStep.prototype.Set;
        module.exports = b2TimeStep;
    },
    s: function(require, module, exports, global) {
        var b2Vec2 = require("1");
        var b2Settings = require("d");
        var b2WorldManifold = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2WorldManifold.prototype.__constructor = function() {
            this.m_points = new Array(b2Settings.b2_maxManifoldPoints);
            for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
                this.m_points[i] = new b2Vec2;
            }
        };
        b2WorldManifold.prototype.__varz = function() {
            this.m_normal = new b2Vec2;
        };
        b2WorldManifold.prototype.Initialize = function(manifold, xfA, radiusA, xfB, radiusB) {
            if (manifold.m_pointCount == 0) {
                return;
            }
            var i = 0;
            var tVec;
            var tMat;
            var normalX;
            var normalY;
            var planePointX;
            var planePointY;
            var clipPointX;
            var clipPointY;
            switch (manifold.m_type) {
              case b2Manifold.e_circles:
                {
                    tMat = xfA.R;
                    tVec = manifold.m_localPoint;
                    var pointAX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
                    var pointAY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
                    tMat = xfB.R;
                    tVec = manifold.m_points[0].m_localPoint;
                    var pointBX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
                    var pointBY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
                    var dX = pointBX - pointAX;
                    var dY = pointBY - pointAY;
                    var d2 = dX * dX + dY * dY;
                    if (d2 > Number.MIN_VALUE * Number.MIN_VALUE) {
                        var d = Math.sqrt(d2);
                        this.m_normal.x = dX / d;
                        this.m_normal.y = dY / d;
                    } else {
                        this.m_normal.x = 1;
                        this.m_normal.y = 0;
                    }
                    var cAX = pointAX + radiusA * this.m_normal.x;
                    var cAY = pointAY + radiusA * this.m_normal.y;
                    var cBX = pointBX - radiusB * this.m_normal.x;
                    var cBY = pointBY - radiusB * this.m_normal.y;
                    this.m_points[0].x = .5 * (cAX + cBX);
                    this.m_points[0].y = .5 * (cAY + cBY);
                }
                break;
              case b2Manifold.e_faceA:
                {
                    tMat = xfA.R;
                    tVec = manifold.m_localPlaneNormal;
                    normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
                    normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
                    tMat = xfA.R;
                    tVec = manifold.m_localPoint;
                    planePointX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
                    planePointY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
                    this.m_normal.x = normalX;
                    this.m_normal.y = normalY;
                    for (i = 0; i < manifold.m_pointCount; i++) {
                        tMat = xfB.R;
                        tVec = manifold.m_points[i].m_localPoint;
                        clipPointX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
                        clipPointY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
                        this.m_points[i].x = clipPointX + .5 * (radiusA - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusB) * normalX;
                        this.m_points[i].y = clipPointY + .5 * (radiusA - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusB) * normalY;
                    }
                }
                break;
              case b2Manifold.e_faceB:
                {
                    tMat = xfB.R;
                    tVec = manifold.m_localPlaneNormal;
                    normalX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
                    normalY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
                    tMat = xfB.R;
                    tVec = manifold.m_localPoint;
                    planePointX = xfB.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
                    planePointY = xfB.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
                    this.m_normal.x = -normalX;
                    this.m_normal.y = -normalY;
                    for (i = 0; i < manifold.m_pointCount; i++) {
                        tMat = xfA.R;
                        tVec = manifold.m_points[i].m_localPoint;
                        clipPointX = xfA.position.x + tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
                        clipPointY = xfA.position.y + tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
                        this.m_points[i].x = clipPointX + .5 * (radiusB - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusA) * normalX;
                        this.m_points[i].y = clipPointY + .5 * (radiusB - (clipPointX - planePointX) * normalX - (clipPointY - planePointY) * normalY - radiusA) * normalY;
                    }
                }
                break;
            }
        };
        b2WorldManifold.prototype.m_normal = new b2Vec2;
        b2WorldManifold.prototype.m_points = null;
        b2WorldManifold.prototype.initialize = b2WorldManifold.prototype.Initialize;
        module.exports = b2WorldManifold;
    },
    t: function(require, module, exports, global) {
        var b2Vec2 = require("1");
        var b2Settings = require("d");
        var b2PositionSolverManifold = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2PositionSolverManifold.prototype.__constructor = function() {
            this.m_normal = new b2Vec2;
            this.m_separations = new Array(b2Settings.b2_maxManifoldPoints);
            this.m_points = new Array(b2Settings.b2_maxManifoldPoints);
            for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
                this.m_points[i] = new b2Vec2;
            }
        };
        b2PositionSolverManifold.prototype.__varz = function() {};
        b2PositionSolverManifold.circlePointA = new b2Vec2;
        b2PositionSolverManifold.circlePointB = new b2Vec2;
        b2PositionSolverManifold.prototype.Initialize = function(cc) {
            b2Settings.b2Assert(cc.pointCount > 0);
            var i = 0;
            var clipPointX;
            var clipPointY;
            var tMat;
            var tVec;
            var planePointX;
            var planePointY;
            switch (cc.type) {
              case b2Manifold.e_circles:
                {
                    tMat = cc.bodyA.m_xf.R;
                    tVec = cc.localPoint;
                    var pointAX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
                    var pointAY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
                    tMat = cc.bodyB.m_xf.R;
                    tVec = cc.points[0].localPoint;
                    var pointBX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
                    var pointBY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
                    var dX = pointBX - pointAX;
                    var dY = pointBY - pointAY;
                    var d2 = dX * dX + dY * dY;
                    if (d2 > Number.MIN_VALUE * Number.MIN_VALUE) {
                        var d = Math.sqrt(d2);
                        this.m_normal.x = dX / d;
                        this.m_normal.y = dY / d;
                    } else {
                        this.m_normal.x = 1;
                        this.m_normal.y = 0;
                    }
                    this.m_points[0].x = .5 * (pointAX + pointBX);
                    this.m_points[0].y = .5 * (pointAY + pointBY);
                    this.m_separations[0] = dX * this.m_normal.x + dY * this.m_normal.y - cc.radius;
                }
                break;
              case b2Manifold.e_faceA:
                {
                    tMat = cc.bodyA.m_xf.R;
                    tVec = cc.localPlaneNormal;
                    this.m_normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
                    this.m_normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
                    tMat = cc.bodyA.m_xf.R;
                    tVec = cc.localPoint;
                    planePointX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
                    planePointY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
                    tMat = cc.bodyB.m_xf.R;
                    for (i = 0; i < cc.pointCount; ++i) {
                        tVec = cc.points[i].localPoint;
                        clipPointX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
                        clipPointY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
                        this.m_separations[i] = (clipPointX - planePointX) * this.m_normal.x + (clipPointY - planePointY) * this.m_normal.y - cc.radius;
                        this.m_points[i].x = clipPointX;
                        this.m_points[i].y = clipPointY;
                    }
                }
                break;
              case b2Manifold.e_faceB:
                {
                    tMat = cc.bodyB.m_xf.R;
                    tVec = cc.localPlaneNormal;
                    this.m_normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
                    this.m_normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
                    tMat = cc.bodyB.m_xf.R;
                    tVec = cc.localPoint;
                    planePointX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
                    planePointY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
                    tMat = cc.bodyA.m_xf.R;
                    for (i = 0; i < cc.pointCount; ++i) {
                        tVec = cc.points[i].localPoint;
                        clipPointX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
                        clipPointY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
                        this.m_separations[i] = (clipPointX - planePointX) * this.m_normal.x + (clipPointY - planePointY) * this.m_normal.y - cc.radius;
                        this.m_points[i].Set(clipPointX, clipPointY);
                    }
                    this.m_normal.x *= -1;
                    this.m_normal.y *= -1;
                }
                break;
            }
        };
        b2PositionSolverManifold.prototype.m_normal = null;
        b2PositionSolverManifold.prototype.m_points = null;
        b2PositionSolverManifold.prototype.m_separations = null;
        b2PositionSolverManifold.prototype.initialize = b2PositionSolverManifold.prototype.Initialize;
        module.exports = b2PositionSolverManifold;
    },
    u: function(require, module, exports, global) {
        var b2ContactConstraintPoint = require("v");
        var b2Vec2 = require("1");
        var b2Mat22 = require("4");
        var b2Settings = require("d");
        var b2ContactConstraint = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2ContactConstraint.prototype.__constructor = function() {
            this.points = new Array(b2Settings.b2_maxManifoldPoints);
            for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
                this.points[i] = new b2ContactConstraintPoint;
            }
        };
        b2ContactConstraint.prototype.__varz = function() {
            this.localPlaneNormal = new b2Vec2;
            this.localPoint = new b2Vec2;
            this.normal = new b2Vec2;
            this.normalMass = new b2Mat22;
            this.K = new b2Mat22;
        };
        b2ContactConstraint.prototype.points = null;
        b2ContactConstraint.prototype.localPlaneNormal = new b2Vec2;
        b2ContactConstraint.prototype.localPoint = new b2Vec2;
        b2ContactConstraint.prototype.normal = new b2Vec2;
        b2ContactConstraint.prototype.normalMass = new b2Mat22;
        b2ContactConstraint.prototype.K = new b2Mat22;
        b2ContactConstraint.prototype.bodyA = null;
        b2ContactConstraint.prototype.bodyB = null;
        b2ContactConstraint.prototype.type = 0;
        b2ContactConstraint.prototype.radius = null;
        b2ContactConstraint.prototype.friction = null;
        b2ContactConstraint.prototype.restitution = null;
        b2ContactConstraint.prototype.pointCount = 0;
        b2ContactConstraint.prototype.manifold = null;
        module.exports = b2ContactConstraint;
    },
    v: function(require, module, exports, global) {
        var b2Vec2 = require("1");
        var b2ContactConstraintPoint = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2ContactConstraintPoint.prototype.__constructor = function() {};
        b2ContactConstraintPoint.prototype.__varz = function() {
            this.localPoint = new b2Vec2;
            this.rA = new b2Vec2;
            this.rB = new b2Vec2;
        };
        b2ContactConstraintPoint.prototype.localPoint = new b2Vec2;
        b2ContactConstraintPoint.prototype.rA = new b2Vec2;
        b2ContactConstraintPoint.prototype.rB = new b2Vec2;
        b2ContactConstraintPoint.prototype.normalImpulse = null;
        b2ContactConstraintPoint.prototype.tangentImpulse = null;
        b2ContactConstraintPoint.prototype.normalMass = null;
        b2ContactConstraintPoint.prototype.tangentMass = null;
        b2ContactConstraintPoint.prototype.equalizedMass = null;
        b2ContactConstraintPoint.prototype.velocityBias = null;
        module.exports = b2ContactConstraintPoint;
    },
    w: function(require, module, exports, global) {
        var b2ContactImpulse = require("x");
        var b2Settings = require("d");
        var b2Math = require("2");
        var b2Island = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2Island.prototype.__constructor = function() {
            this.m_bodies = new Array;
            this.m_contacts = new Array;
            this.m_joints = new Array;
        };
        b2Island.prototype.__varz = function() {};
        b2Island.s_impulse = new b2ContactImpulse;
        b2Island.prototype.Initialize = function(bodyCapacity, contactCapacity, jointCapacity, allocator, listener, contactSolver) {
            var i = 0;
            this.m_bodyCapacity = bodyCapacity;
            this.m_contactCapacity = contactCapacity;
            this.m_jointCapacity = jointCapacity;
            this.m_bodyCount = 0;
            this.m_contactCount = 0;
            this.m_jointCount = 0;
            this.m_allocator = allocator;
            this.m_listener = listener;
            this.m_contactSolver = contactSolver;
            for (i = this.m_bodies.length; i < bodyCapacity; i++) this.m_bodies[i] = null;
            for (i = this.m_contacts.length; i < contactCapacity; i++) this.m_contacts[i] = null;
            for (i = this.m_joints.length; i < jointCapacity; i++) this.m_joints[i] = null;
        };
        b2Island.prototype.Clear = function() {
            this.m_bodyCount = 0;
            this.m_contactCount = 0;
            this.m_jointCount = 0;
        };
        b2Island.prototype.Solve = function(step, gravity, allowSleep) {
            var i = 0;
            var j = 0;
            var b;
            var joint;
            for (i = 0; i < this.m_bodyCount; ++i) {
                b = this.m_bodies[i];
                if (b.GetType() != b2Body.b2_dynamicBody) continue;
                b.m_linearVelocity.x += step.dt * (gravity.x + b.m_invMass * b.m_force.x);
                b.m_linearVelocity.y += step.dt * (gravity.y + b.m_invMass * b.m_force.y);
                b.m_angularVelocity += step.dt * b.m_invI * b.m_torque;
                b.m_linearVelocity.Multiply(b2Math.Clamp(1 - step.dt * b.m_linearDamping, 0, 1));
                b.m_angularVelocity *= b2Math.Clamp(1 - step.dt * b.m_angularDamping, 0, 1);
            }
            this.m_contactSolver.Initialize(step, this.m_contacts, this.m_contactCount, this.m_allocator);
            var contactSolver = this.m_contactSolver;
            contactSolver.InitVelocityConstraints(step);
            for (i = 0; i < this.m_jointCount; ++i) {
                joint = this.m_joints[i];
                joint.InitVelocityConstraints(step);
            }
            for (i = 0; i < step.velocityIterations; ++i) {
                for (j = 0; j < this.m_jointCount; ++j) {
                    joint = this.m_joints[j];
                    joint.SolveVelocityConstraints(step);
                }
                contactSolver.SolveVelocityConstraints();
            }
            for (i = 0; i < this.m_jointCount; ++i) {
                joint = this.m_joints[i];
                joint.FinalizeVelocityConstraints();
            }
            contactSolver.FinalizeVelocityConstraints();
            for (i = 0; i < this.m_bodyCount; ++i) {
                b = this.m_bodies[i];
                if (b.GetType() == b2Body.b2_staticBody) continue;
                var translationX = step.dt * b.m_linearVelocity.x;
                var translationY = step.dt * b.m_linearVelocity.y;
                if (translationX * translationX + translationY * translationY > b2Settings.b2_maxTranslationSquared) {
                    b.m_linearVelocity.Normalize();
                    b.m_linearVelocity.x *= b2Settings.b2_maxTranslation * step.inv_dt;
                    b.m_linearVelocity.y *= b2Settings.b2_maxTranslation * step.inv_dt;
                }
                var rotation = step.dt * b.m_angularVelocity;
                if (rotation * rotation > b2Settings.b2_maxRotationSquared) {
                    if (b.m_angularVelocity < 0) {
                        b.m_angularVelocity = -b2Settings.b2_maxRotation * step.inv_dt;
                    } else {
                        b.m_angularVelocity = b2Settings.b2_maxRotation * step.inv_dt;
                    }
                }
                b.m_sweep.c0.SetV(b.m_sweep.c);
                b.m_sweep.a0 = b.m_sweep.a;
                b.m_sweep.c.x += step.dt * b.m_linearVelocity.x;
                b.m_sweep.c.y += step.dt * b.m_linearVelocity.y;
                b.m_sweep.a += step.dt * b.m_angularVelocity;
                b.SynchronizeTransform();
            }
            for (i = 0; i < step.positionIterations; ++i) {
                var contactsOkay = contactSolver.SolvePositionConstraints(b2Settings.b2_contactBaumgarte);
                var jointsOkay = true;
                for (j = 0; j < this.m_jointCount; ++j) {
                    joint = this.m_joints[j];
                    var jointOkay = joint.SolvePositionConstraints(b2Settings.b2_contactBaumgarte);
                    jointsOkay = jointsOkay && jointOkay;
                }
                if (contactsOkay && jointsOkay) {
                    break;
                }
            }
            this.Report(contactSolver.m_constraints);
            if (allowSleep) {
                var minSleepTime = Number.MAX_VALUE;
                var linTolSqr = b2Settings.b2_linearSleepTolerance * b2Settings.b2_linearSleepTolerance;
                var angTolSqr = b2Settings.b2_angularSleepTolerance * b2Settings.b2_angularSleepTolerance;
                for (i = 0; i < this.m_bodyCount; ++i) {
                    b = this.m_bodies[i];
                    if (b.GetType() == b2Body.b2_staticBody) {
                        continue;
                    }
                    if ((b.m_flags & b2Body.e_allowSleepFlag) == 0) {
                        b.m_sleepTime = 0;
                        minSleepTime = 0;
                    }
                    if ((b.m_flags & b2Body.e_allowSleepFlag) == 0 || b.m_angularVelocity * b.m_angularVelocity > angTolSqr || b2Math.Dot(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr) {
                        b.m_sleepTime = 0;
                        minSleepTime = 0;
                    } else {
                        b.m_sleepTime += step.dt;
                        minSleepTime = b2Math.Min(minSleepTime, b.m_sleepTime);
                    }
                }
                if (minSleepTime >= b2Settings.b2_timeToSleep) {
                    for (i = 0; i < this.m_bodyCount; ++i) {
                        b = this.m_bodies[i];
                        b.SetAwake(false);
                    }
                }
            }
        };
        b2Island.prototype.SolveTOI = function(subStep) {
            var i = 0;
            var j = 0;
            this.m_contactSolver.Initialize(subStep, this.m_contacts, this.m_contactCount, this.m_allocator);
            var contactSolver = this.m_contactSolver;
            for (i = 0; i < this.m_jointCount; ++i) {
                this.m_joints[i].InitVelocityConstraints(subStep);
            }
            for (i = 0; i < subStep.velocityIterations; ++i) {
                contactSolver.SolveVelocityConstraints();
                for (j = 0; j < this.m_jointCount; ++j) {
                    this.m_joints[j].SolveVelocityConstraints(subStep);
                }
            }
            for (i = 0; i < this.m_bodyCount; ++i) {
                var b = this.m_bodies[i];
                if (b.GetType() == b2Body.b2_staticBody) continue;
                var translationX = subStep.dt * b.m_linearVelocity.x;
                var translationY = subStep.dt * b.m_linearVelocity.y;
                if (translationX * translationX + translationY * translationY > b2Settings.b2_maxTranslationSquared) {
                    b.m_linearVelocity.Normalize();
                    b.m_linearVelocity.x *= b2Settings.b2_maxTranslation * subStep.inv_dt;
                    b.m_linearVelocity.y *= b2Settings.b2_maxTranslation * subStep.inv_dt;
                }
                var rotation = subStep.dt * b.m_angularVelocity;
                if (rotation * rotation > b2Settings.b2_maxRotationSquared) {
                    if (b.m_angularVelocity < 0) {
                        b.m_angularVelocity = -b2Settings.b2_maxRotation * subStep.inv_dt;
                    } else {
                        b.m_angularVelocity = b2Settings.b2_maxRotation * subStep.inv_dt;
                    }
                }
                b.m_sweep.c0.SetV(b.m_sweep.c);
                b.m_sweep.a0 = b.m_sweep.a;
                b.m_sweep.c.x += subStep.dt * b.m_linearVelocity.x;
                b.m_sweep.c.y += subStep.dt * b.m_linearVelocity.y;
                b.m_sweep.a += subStep.dt * b.m_angularVelocity;
                b.SynchronizeTransform();
            }
            var k_toiBaumgarte = .75;
            for (i = 0; i < subStep.positionIterations; ++i) {
                var contactsOkay = contactSolver.SolvePositionConstraints(k_toiBaumgarte);
                var jointsOkay = true;
                for (j = 0; j < this.m_jointCount; ++j) {
                    var jointOkay = this.m_joints[j].SolvePositionConstraints(b2Settings.b2_contactBaumgarte);
                    jointsOkay = jointsOkay && jointOkay;
                }
                if (contactsOkay && jointsOkay) {
                    break;
                }
            }
            this.Report(contactSolver.m_constraints);
        };
        b2Island.prototype.Report = function(constraints) {
            if (this.m_listener == null) {
                return;
            }
            for (var i = 0; i < this.m_contactCount; ++i) {
                var c = this.m_contacts[i];
                var cc = constraints[i];
                for (var j = 0; j < cc.pointCount; ++j) {
                    b2Island.s_impulse.normalImpulses[j] = cc.points[j].normalImpulse;
                    b2Island.s_impulse.tangentImpulses[j] = cc.points[j].tangentImpulse;
                }
                this.m_listener.PostSolve(c, b2Island.s_impulse);
            }
        };
        b2Island.prototype.AddBody = function(body) {
            body.m_islandIndex = this.m_bodyCount;
            this.m_bodies[this.m_bodyCount++] = body;
        };
        b2Island.prototype.AddContact = function(contact) {
            this.m_contacts[this.m_contactCount++] = contact;
        };
        b2Island.prototype.AddJoint = function(joint) {
            this.m_joints[this.m_jointCount++] = joint;
        };
        b2Island.prototype.m_allocator = null;
        b2Island.prototype.m_listener = null;
        b2Island.prototype.m_contactSolver = null;
        b2Island.prototype.m_bodies = null;
        b2Island.prototype.m_contacts = null;
        b2Island.prototype.m_joints = null;
        b2Island.prototype.m_bodyCount = 0;
        b2Island.prototype.m_jointCount = 0;
        b2Island.prototype.m_contactCount = 0;
        b2Island.prototype.m_bodyCapacity = 0;
        b2Island.prototype.m_contactCapacity = 0;
        b2Island.prototype.m_jointCapacity = 0;
        b2Island.prototype.initialize = b2Island.prototype.Initialize;
        b2Island.prototype.clear = b2Island.prototype.Clear;
        b2Island.prototype.solve = b2Island.prototype.Solve;
        b2Island.prototype.solveTOI = b2Island.prototype.SolveTOI;
        b2Island.prototype.report = b2Island.prototype.Report;
        b2Island.prototype.addBody = b2Island.prototype.AddBody;
        b2Island.prototype.addContact = b2Island.prototype.AddContact;
        b2Island.prototype.addJoint = b2Island.prototype.AddJoint;
        module.exports = b2Island;
    },
    x: function(require, module, exports, global) {
        var b2Settings = require("d");
        var b2ContactImpulse = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2ContactImpulse.prototype.__constructor = function() {};
        b2ContactImpulse.prototype.__varz = function() {
            this.normalImpulses = new Array(b2Settings.b2_maxManifoldPoints);
            this.tangentImpulses = new Array(b2Settings.b2_maxManifoldPoints);
        };
        b2ContactImpulse.prototype.normalImpulses = new Array(b2Settings.b2_maxManifoldPoints);
        b2ContactImpulse.prototype.tangentImpulses = new Array(b2Settings.b2_maxManifoldPoints);
        module.exports = b2ContactImpulse;
    },
    y: function(require, module, exports, global) {
        var b2Math = require("2");
        var b2Color = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2Color.prototype.__constructor = function(rr, gg, bb) {
            this._r = parseInt(255 * b2Math.Clamp(rr, 0, 1));
            this._g = parseInt(255 * b2Math.Clamp(gg, 0, 1));
            this._b = parseInt(255 * b2Math.Clamp(bb, 0, 1));
        };
        b2Color.prototype.__varz = function() {};
        b2Color.prototype.Set = function(rr, gg, bb) {
            this._r = parseInt(255 * b2Math.Clamp(rr, 0, 1));
            this._g = parseInt(255 * b2Math.Clamp(gg, 0, 1));
            this._b = parseInt(255 * b2Math.Clamp(bb, 0, 1));
        };
        b2Color.prototype.__defineGetter__("r", function() {
            return this._r;
        });
        b2Color.prototype.__defineSetter__("r", function(rr) {
            this._r = parseInt(255 * b2Math.Clamp(rr, 0, 1));
        });
        b2Color.prototype.__defineGetter__("g", function() {
            return this._g;
        });
        b2Color.prototype.__defineSetter__("g", function(gg) {
            this._g = parseInt(255 * b2Math.Clamp(gg, 0, 1));
        });
        b2Color.prototype.__defineGetter__("b", function() {
            return this._b;
        });
        b2Color.prototype.__defineSetter__("b", function(bb) {
            this._b = parseInt(255 * b2Math.Clamp(bb, 0, 1));
        });
        b2Color.prototype.__defineGetter__("color", function() {
            return this._r << 16 | this._g << 8 | this._b;
        });
        b2Color.prototype._r = 0;
        b2Color.prototype._g = 0;
        b2Color.prototype._b = 0;
        b2Color.prototype.set = b2Color.prototype.Set;
        module.exports = b2Color;
    },
    z: function(require, module, exports, global) {
        var b2Vec2 = require("1");
        var b2RayCastOutput = function() {
            this.__varz();
            this.__constructor.apply(this, arguments);
        };
        b2RayCastOutput.prototype.__constructor = function() {};
        b2RayCastOutput.prototype.__varz = function() {
            this.normal = new b2Vec2;
        };
        b2RayCastOutput.prototype.normal = new b2Vec2;
        b2RayCastOutput.prototype.fraction = null;
        module.exports = b2RayCastOutput;
    },
    "10": function(require, module, exports, global) {
        var b2Vec2 = require("1");
        var b2Mat22 = require("4");
        var b2Transform = require("3");
        var b2MassData = require("b");
        var b2Settings = require("d");
        var b2Math = require("2");
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
        b2PolygonShape.prototype.validate = b2PolygonShape.prototype.Validate;
        b2PolygonShape.prototype.reserve = b2PolygonShape.prototype.Reserve;
        b2PolygonShape.prototype.copy = b2PolygonShape.prototype.Copy;
        b2PolygonShape.prototype.set = b2PolygonShape.prototype.Set;
        b2PolygonShape.prototype.setAsArray = b2PolygonShape.prototype.SetAsArray;
        b2PolygonShape.prototype.setAsVector = b2PolygonShape.prototype.SetAsVector;
        b2PolygonShape.prototype.setAsBox = b2PolygonShape.prototype.SetAsBox;
        b2PolygonShape.prototype.setAsOrientedBox = b2PolygonShape.prototype.SetAsOrientedBox;
        b2PolygonShape.prototype.setAsEdge = b2PolygonShape.prototype.SetAsEdge;
        b2PolygonShape.prototype.testPoint = b2PolygonShape.prototype.TestPoint;
        b2PolygonShape.prototype.rayCast = b2PolygonShape.prototype.RayCast;
        b2PolygonShape.prototype.computeAABB = b2PolygonShape.prototype.ComputeAABB;
        b2PolygonShape.prototype.computeMass = b2PolygonShape.prototype.ComputeMass;
        b2PolygonShape.prototype.computeSubmergedArea = b2PolygonShape.prototype.ComputeSubmergedArea;
        b2PolygonShape.prototype.getVertexCount = b2PolygonShape.prototype.GetVertexCount;
        b2PolygonShape.prototype.getVertices = b2PolygonShape.prototype.GetVertices;
        b2PolygonShape.prototype.getNormals = b2PolygonShape.prototype.GetNormals;
        b2PolygonShape.prototype.getSupport = b2PolygonShape.prototype.GetSupport;
        b2PolygonShape.prototype.getSupportVertex = b2PolygonShape.prototype.GetSupportVertex;
        module.exports = b2PolygonShape;
    },
    "11": function(require, module, exports, global) {
        var b2Vec2 = require("1");
        var b2Settings = require("d");
        var b2Math = require("2");
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
        b2CircleShape.prototype.copy = b2CircleShape.prototype.Copy;
        b2CircleShape.prototype.set = b2CircleShape.prototype.Set;
        b2CircleShape.prototype.testPoint = b2CircleShape.prototype.TestPoint;
        b2CircleShape.prototype.rayCast = b2CircleShape.prototype.RayCast;
        b2CircleShape.prototype.computeAABB = b2CircleShape.prototype.ComputeAABB;
        b2CircleShape.prototype.computeMass = b2CircleShape.prototype.ComputeMass;
        b2CircleShape.prototype.computeSubmergedArea = b2CircleShape.prototype.ComputeSubmergedArea;
        b2CircleShape.prototype.getLocalPosition = b2CircleShape.prototype.GetLocalPosition;
        b2CircleShape.prototype.setLocalPosition = b2CircleShape.prototype.SetLocalPosition;
        b2CircleShape.prototype.getRadius = b2CircleShape.prototype.GetRadius;
        b2CircleShape.prototype.setRadius = b2CircleShape.prototype.SetRadius;
        module.exports = b2CircleShape;
    },
    "12": function(require, module, exports, global) {
        var b2Vec2 = require("1");
        var b2Color = require("y");
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
        b2DebugDraw.prototype.setFlags = b2DebugDraw.prototype.SetFlags;
        b2DebugDraw.prototype.getFlags = b2DebugDraw.prototype.GetFlags;
        b2DebugDraw.prototype.appendFlags = b2DebugDraw.prototype.AppendFlags;
        b2DebugDraw.prototype.clearFlags = b2DebugDraw.prototype.ClearFlags;
        b2DebugDraw.prototype.setSprite = b2DebugDraw.prototype.SetSprite;
        b2DebugDraw.prototype.getSprite = b2DebugDraw.prototype.GetSprite;
        b2DebugDraw.prototype.setDrawScale = b2DebugDraw.prototype.SetDrawScale;
        b2DebugDraw.prototype.getDrawScale = b2DebugDraw.prototype.GetDrawScale;
        b2DebugDraw.prototype.setLineThickness = b2DebugDraw.prototype.SetLineThickness;
        b2DebugDraw.prototype.getLineThickness = b2DebugDraw.prototype.GetLineThickness;
        b2DebugDraw.prototype.setAlpha = b2DebugDraw.prototype.SetAlpha;
        b2DebugDraw.prototype.getAlpha = b2DebugDraw.prototype.GetAlpha;
        b2DebugDraw.prototype.setFillAlpha = b2DebugDraw.prototype.SetFillAlpha;
        b2DebugDraw.prototype.getFillAlpha = b2DebugDraw.prototype.GetFillAlpha;
        b2DebugDraw.prototype.setXFormScale = b2DebugDraw.prototype.SetXFormScale;
        b2DebugDraw.prototype.getXFormScale = b2DebugDraw.prototype.GetXFormScale;
        b2DebugDraw.prototype.clear = b2DebugDraw.prototype.Clear;
        b2DebugDraw.prototype.y = b2DebugDraw.prototype.Y;
        b2DebugDraw.prototype.toWorldPoint = b2DebugDraw.prototype.ToWorldPoint;
        b2DebugDraw.prototype.colorStyle = b2DebugDraw.prototype.ColorStyle;
        b2DebugDraw.prototype.drawPolygon = b2DebugDraw.prototype.DrawPolygon;
        b2DebugDraw.prototype.drawSolidPolygon = b2DebugDraw.prototype.DrawSolidPolygon;
        b2DebugDraw.prototype.drawCircle = b2DebugDraw.prototype.DrawCircle;
        b2DebugDraw.prototype.drawSolidCircle = b2DebugDraw.prototype.DrawSolidCircle;
        b2DebugDraw.prototype.drawSegment = b2DebugDraw.prototype.DrawSegment;
        b2DebugDraw.prototype.drawTransform = b2DebugDraw.prototype.DrawTransform;
        module.exports = b2DebugDraw;
    }
});
