// requires 
var b2Math = require('./math.js'); 
var b2Vec2 = function(x_, y_) {
    if(arguments.length == 2) {
        this.x=x_; this.y=y_;
}
}
// static methods
b2Vec2.Make = function (x_, y_) {
		return new b2Vec2(x_, y_);
	}
// static attributes
// methods
b2Vec2.prototype.SetZero = function () { this.x = 0.0; this.y = 0.0; }
b2Vec2.prototype.Set = function (x_, y_) {this.x=x_; this.y=y_;}
b2Vec2.prototype.SetV = function (v) {this.x=v.x; this.y=v.y;}
b2Vec2.prototype.GetNegative = function () { return new b2Vec2(-this.x, -this.y); }
b2Vec2.prototype.NegativeSelf = function () { this.x = -this.x; this.y = -this.y; }
b2Vec2.prototype.Copy = function () {
		return new b2Vec2(this.x,this.y);
	}
b2Vec2.prototype.Add = function (v) {
		this.x += v.x; this.y += v.y;
	}
b2Vec2.prototype.Subtract = function (v) {
		this.x -= v.x; this.y -= v.y;
	}
b2Vec2.prototype.Multiply = function (a) {
		this.x *= a; this.y *= a;
	}
b2Vec2.prototype.MulM = function (A) {
		var tX = this.x;
		this.x = A.col1.x * tX + A.col2.x * this.y;
		this.y = A.col1.y * tX + A.col2.y * this.y;
	}
b2Vec2.prototype.MulTM = function (A) {
		var tX = b2Math.Dot(this, A.col1);
		this.y = b2Math.Dot(this, A.col2);
		this.x = tX;
	}
b2Vec2.prototype.CrossVF = function (s) {
		var tX = this.x;
		this.x = s * this.y;
		this.y = -s * tX;
	}
b2Vec2.prototype.CrossFV = function (s) {
		var tX = this.x;
		this.x = -s * this.y;
		this.y = s * tX;
	}
b2Vec2.prototype.MinV = function (b) {
		this.x = this.x < b.x ? this.x : b.x;
		this.y = this.y < b.y ? this.y : b.y;
	}
b2Vec2.prototype.MaxV = function (b) {
		this.x = this.x > b.x ? this.x : b.x;
		this.y = this.y > b.y ? this.y : b.y;
	}
b2Vec2.prototype.Abs = function () {
		if (this.x < 0) this.x = -this.x;
		if (this.y < 0) this.y = -this.y;
	}
b2Vec2.prototype.Length = function () {
		return Math.sqrt(this.x * this.x + this.y * this.y);
	}
b2Vec2.prototype.LengthSquared = function () {
		return (this.x * this.x + this.y * this.y);
	}
b2Vec2.prototype.Normalize = function () {
		var length = Math.sqrt(this.x * this.x + this.y * this.y);
		if (length < Number.MIN_VALUE)
		{
			return 0.0;
		}
		var invLength = 1.0 / length;
		this.x *= invLength;
		this.y *= invLength;
		
		return length;
	}
b2Vec2.prototype.IsValid = function () {
		return b2Math.IsValid(this.x) && b2Math.IsValid(this.y);
	}
// attributes
b2Vec2.prototype.x =  0;
b2Vec2.prototype.y =  0;// aliases  
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
// exports  
module.exports =  b2Vec2;
