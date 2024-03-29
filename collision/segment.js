// requires 
var b2Vec2 = require('../common/math/vec2.js'); 
var b2Segment = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Segment.prototype.__constructor = function(){}
b2Segment.prototype.__varz = function(){
this.p1 =  new b2Vec2();
this.p2 =  new b2Vec2();
}
// static methods
// static attributes
// methods
b2Segment.prototype.TestSegment = function (lambda, 
								normal, 
								segment, 
								maxLambda) {
		
		var s = segment.p1;
		
		var rX = segment.p2.x - s.x;
		var rY = segment.p2.y - s.y;
		
		var dX = this.p2.x - this.p1.x;
		var dY = this.p2.y - this.p1.y;
		
		var nX = dY;
		var nY = -dX;
		
		var k_slop = 100.0 * Number.MIN_VALUE;
		
		var denom = -(rX*nX + rY*nY);
		
		
		if (denom > k_slop)
		{
			
			
			var bX = s.x - this.p1.x;
			var bY = s.y - this.p1.y;
			
			var a = (bX*nX + bY*nY);
			
			if (0.0 <= a && a <= maxLambda * denom)
			{
				var mu2 = -rX * bY + rY * bX;
				
				
				if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0 + k_slop))
				{
					a /= denom;
					
					var nLen = Math.sqrt(nX*nX + nY*nY);
					nX /= nLen;
					nY /= nLen;
					
					lambda[0] = a;
					
					normal.Set(nX, nY);
					return true;
				}
			}
		}
		
		return false;
	}
b2Segment.prototype.Extend = function (aabb) {
		this.ExtendForward(aabb);
		this.ExtendBackward(aabb);
	}
b2Segment.prototype.ExtendForward = function (aabb) {
		var dX = this.p2.x-this.p1.x;
		var dY = this.p2.y-this.p1.y;
		
		var lambda = Math.min(	dX>0?(aabb.upperBound.x-this.p1.x)/dX: dX<0?(aabb.lowerBound.x-this.p1.x)/dX:Number.POSITIVE_INFINITY,
										dY>0?(aabb.upperBound.y-this.p1.y)/dY: dY<0?(aabb.lowerBound.y-this.p1.y)/dY:Number.POSITIVE_INFINITY);
		
		this.p2.x = this.p1.x + dX * lambda;
		this.p2.y = this.p1.y + dY * lambda;
		
	}
b2Segment.prototype.ExtendBackward = function (aabb) {
		var dX = -this.p2.x+this.p1.x;
		var dY = -this.p2.y+this.p1.y;
		
		var lambda = Math.min(	dX>0?(aabb.upperBound.x-this.p2.x)/dX: dX<0?(aabb.lowerBound.x-this.p2.x)/dX:Number.POSITIVE_INFINITY,
										dY>0?(aabb.upperBound.y-this.p2.y)/dY: dY<0?(aabb.lowerBound.y-this.p2.y)/dY:Number.POSITIVE_INFINITY);
		
		this.p1.x = this.p2.x + dX * lambda;
		this.p1.y = this.p2.y + dY * lambda;
		
	}
// attributes
b2Segment.prototype.p1 =  new b2Vec2();
b2Segment.prototype.p2 =  new b2Vec2();// aliases  
b2Segment.prototype.testSegment = b2Segment.prototype.TestSegment; 
b2Segment.prototype.extend = b2Segment.prototype.Extend; 
b2Segment.prototype.extendForward = b2Segment.prototype.ExtendForward; 
b2Segment.prototype.extendBackward = b2Segment.prototype.ExtendBackward; 
// exports  
module.exports =  b2Segment;
