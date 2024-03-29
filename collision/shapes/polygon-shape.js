// requires 
var b2Vec2 = require('../../common/math/vec2.js'); 
var b2Mat22 = require('../../common/math/mat22.js'); 
var b2Transform = require('../../common/math/transform.js'); 
var b2MassData = require('./mass-data.js'); 
var b2Settings = require('../../common/settings.js'); 
var b2Math = require('../../common/math/math.js'); 
var b2PolygonShape = function() {
b2Shape.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2PolygonShape.prototype, b2Shape.prototype)
b2PolygonShape.prototype._super = b2Shape.prototype;
b2PolygonShape.prototype.__constructor = function () {
		
		
		this._super.__constructor.apply(this, arguments);
		
		
		this.m_type = b2Shape.e_polygonShape;
		
		this.m_centroid = new b2Vec2();
		this.m_vertices = new Array();
		this.m_normals = new Array();
	}
b2PolygonShape.prototype.__varz = function(){
}
// static methods
b2PolygonShape.AsArray = function (vertices, vertexCount) {
		var polygonShape = new b2PolygonShape();
		polygonShape.SetAsArray(vertices, vertexCount);
		return polygonShape;
	}
b2PolygonShape.AsVector = function (vertices, vertexCount) {
		var polygonShape = new b2PolygonShape();
		polygonShape.SetAsVector(vertices, vertexCount);
		return polygonShape;
	}
b2PolygonShape.AsBox = function (hx, hy) {
		var polygonShape = new b2PolygonShape();
		polygonShape.SetAsBox(hx, hy);
		return polygonShape;
	}
b2PolygonShape.AsOrientedBox = function (hx, hy, center , angle ) {
		var polygonShape = new b2PolygonShape();
		polygonShape.SetAsOrientedBox(hx, hy, center, angle);
		return polygonShape;
	}
b2PolygonShape.AsEdge = function (v1, v2) {
		var polygonShape = new b2PolygonShape();
		polygonShape.SetAsEdge(v1, v2);
		return polygonShape;
	}
b2PolygonShape.ComputeCentroid = function (vs, count) {
		
		
		
		var c = new b2Vec2();
		var area = 0.0;
		
		
		
		
		var p1X = 0.0;
		var p1Y = 0.0;
	
		
		var inv3 = 1.0 / 3.0;
		
		for (var i = 0; i < count; ++i)
		{
			
			
				
			
			var p2 = vs[i];
			
			var p3 = i + 1 < count ? vs[parseInt(i+1)] : vs[0];
			
			
			var e1X = p2.x - p1X;
			var e1Y = p2.y - p1Y;
			
			var e2X = p3.x - p1X;
			var e2Y = p3.y - p1Y;
			
			
			var D = (e1X * e2Y - e1Y * e2X);
			
			
			var triangleArea = 0.5 * D;
			area += triangleArea;
			
			
			
			c.x += triangleArea * inv3 * (p1X + p2.x + p3.x);
			c.y += triangleArea * inv3 * (p1Y + p2.y + p3.y);
		}
		
		
		
		
		c.x *= 1.0 / area;
		c.y *= 1.0 / area;
		return c;
	}
b2PolygonShape.ComputeOBB = function (obb, vs, count) {
		var i = 0;
		var p = new Array(count + 1);
		for (i = 0; i < count; ++i)
		{
			p[i] = vs[i];
		}
		p[count] = p[0];
		
		var minArea = Number.MAX_VALUE;
		
		for (i = 1; i <= count; ++i)
		{
			var root = p[parseInt(i-1)];
			
			var uxX = p[i].x - root.x;
			var uxY = p[i].y - root.y;
			
			var length = Math.sqrt(uxX*uxX + uxY*uxY);
			uxX /= length;
			uxY /= length;
			
			
			var uyX = -uxY;
			var uyY = uxX;
			
			var lowerX = Number.MAX_VALUE;
			var lowerY = Number.MAX_VALUE;
			
			var upperX = -Number.MAX_VALUE;
			var upperY = -Number.MAX_VALUE;
			
			for (var j = 0; j < count; ++j)
			{
				
				var dX = p[j].x - root.x;
				var dY = p[j].y - root.y;
				
				
				var rX = (uxX*dX + uxY*dY);
				
				var rY = (uyX*dX + uyY*dY);
				
				if (rX < lowerX) lowerX = rX;
				if (rY < lowerY) lowerY = rY;
				
				if (rX > upperX) upperX = rX;
				if (rY > upperY) upperY = rY;
			}
			
			var area = (upperX - lowerX) * (upperY - lowerY);
			if (area < 0.95 * minArea)
			{
				minArea = area;
				
				obb.R.col1.x = uxX;
				obb.R.col1.y = uxY;
				
				obb.R.col2.x = uyX;
				obb.R.col2.y = uyY;
				
				var centerX = 0.5 * (lowerX + upperX);
				var centerY = 0.5 * (lowerY + upperY);
				
				var tMat = obb.R;
				obb.center.x = root.x + (tMat.col1.x * centerX + tMat.col2.x * centerY);
				obb.center.y = root.y + (tMat.col1.y * centerX + tMat.col2.y * centerY);
				
				obb.extents.x = 0.5 * (upperX - lowerX);
				obb.extents.y = 0.5 * (upperY - lowerY);
			}
		}
		
		
	}
// static attributes
b2PolygonShape.s_mat =  new b2Mat22();
// methods
b2PolygonShape.prototype.Validate = function () {
		
		return false;
	}
b2PolygonShape.prototype.Reserve = function (count) {
		for (var i = this.m_vertices.length; i < count; i++)
		{
			this.m_vertices[i] = new b2Vec2();
			this.m_normals[i] = new b2Vec2();
		}
	}
b2PolygonShape.prototype.Copy = function () {
		var s = new b2PolygonShape();
		s.Set(this);
		return s;
	}
b2PolygonShape.prototype.Set = function (other) {
		this._super.Set.apply(this, [other]);
		if (isInstanceOf(other, b2PolygonShape))
		{
			var other2 = other;
			this.m_centroid.SetV(other2.m_centroid);
			this.m_vertexCount = other2.m_vertexCount;
			this.Reserve(this.m_vertexCount);
			for (var i = 0; i < this.m_vertexCount; i++)
			{
				this.m_vertices[i].SetV(other2.m_vertices[i]);
				this.m_normals[i].SetV(other2.m_normals[i]);
			}
		}
	}
b2PolygonShape.prototype.SetAsArray = function (vertices, vertexCount ) {
		var v = new Array();
		for(var i=0, tVec=null;i<vertices.length, tVec=vertices[i]; i++)
		{
			v.push(tVec);
		}
		this.SetAsVector(v, vertexCount);
	}
b2PolygonShape.prototype.SetAsVector = function (vertices, vertexCount ) {
		if (typeof vertexCount == "undefined")
			vertexCount = vertices.length;
			
		b2Settings.b2Assert(2 <= vertexCount);
		this.m_vertexCount = vertexCount;
		
		this.Reserve(vertexCount);
		
		var i = 0;
		
		
		for (i = 0; i < this.m_vertexCount; i++)
		{
			this.m_vertices[i].SetV(vertices[i]);
		}
		
		
		for (i = 0; i < this.m_vertexCount; ++i)
		{
			var i1 = i;
			var i2 = i + 1 < this.m_vertexCount ? i + 1 : 0;
			var edge = b2Math.SubtractVV(this.m_vertices[i2], this.m_vertices[i1]);
			b2Settings.b2Assert(edge.LengthSquared() > Number.MIN_VALUE );
			this.m_normals[i].SetV(b2Math.CrossVF(edge, 1.0));
			this.m_normals[i].Normalize();
		}
		

		
		
		
		
			
			
			
			
			
				
				
				
					
				
				
				
				
				
				
				
			
		


		
		this.m_centroid = b2PolygonShape.ComputeCentroid(this.m_vertices, this.m_vertexCount);
	}
b2PolygonShape.prototype.SetAsBox = function (hx, hy) {
		this.m_vertexCount = 4;
		this.Reserve(4);
		this.m_vertices[0].Set(-hx, -hy);
		this.m_vertices[1].Set( hx, -hy);
		this.m_vertices[2].Set( hx, hy);
		this.m_vertices[3].Set(-hx, hy);
		this.m_normals[0].Set(0.0, -1.0);
		this.m_normals[1].Set(1.0, 0.0);
		this.m_normals[2].Set(0.0, 1.0);
		this.m_normals[3].Set(-1.0, 0.0);
		this.m_centroid.SetZero();
	}
b2PolygonShape.prototype.SetAsOrientedBox = function (hx, hy, center , angle ) {
		this.m_vertexCount = 4;
		this.Reserve(4);
		this.m_vertices[0].Set(-hx, -hy);
		this.m_vertices[1].Set( hx, -hy);
		this.m_vertices[2].Set( hx, hy);
		this.m_vertices[3].Set(-hx, hy);
		this.m_normals[0].Set(0.0, -1.0);
		this.m_normals[1].Set(1.0, 0.0);
		this.m_normals[2].Set(0.0, 1.0);
		this.m_normals[3].Set(-1.0, 0.0);
		this.m_centroid = center;

		var xf = new b2Transform();
		xf.position = center;
		xf.R.Set(angle);

		
		for (var i = 0; i < this.m_vertexCount; ++i)
		{
			this.m_vertices[i] = b2Math.MulX(xf, this.m_vertices[i]);
			this.m_normals[i] = b2Math.MulMV(xf.R, this.m_normals[i]);
		}
	}
b2PolygonShape.prototype.SetAsEdge = function (v1, v2) {
		this.m_vertexCount = 2;
		this.Reserve(2);
		this.m_vertices[0].SetV(v1);
		this.m_vertices[1].SetV(v2);
		this.m_centroid.x = 0.5 * (v1.x + v2.x);
		this.m_centroid.y = 0.5 * (v1.y + v2.y);
		this.m_normals[0] = b2Math.CrossVF(b2Math.SubtractVV(v2, v1), 1.0);
		this.m_normals[0].Normalize();
		this.m_normals[1].x = -this.m_normals[0].x;
		this.m_normals[1].y = -this.m_normals[0].y;
	}
b2PolygonShape.prototype.TestPoint = function (xf, p) {
		var tVec;
		
		
		var tMat = xf.R;
		var tX = p.x - xf.position.x;
		var tY = p.y - xf.position.y;
		var pLocalX = (tX*tMat.col1.x + tY*tMat.col1.y);
		var pLocalY = (tX*tMat.col2.x + tY*tMat.col2.y);
		
		for (var i = 0; i < this.m_vertexCount; ++i)
		{
			
			tVec = this.m_vertices[i];
			tX = pLocalX - tVec.x;
			tY = pLocalY - tVec.y;
			tVec = this.m_normals[i];
			var dot = (tVec.x * tX + tVec.y * tY);
			if (dot > 0.0)
			{
				return false;
			}
		}
		
		return true;
	}
b2PolygonShape.prototype.RayCast = function (output, input, transform) {
		var lower = 0.0;
		var upper = input.maxFraction;
		
		var tX;
		var tY;
		var tMat;
		var tVec;
		
		
		
		tX = input.p1.x - transform.position.x;
		tY = input.p1.y - transform.position.y;
		tMat = transform.R;
		var p1X = (tX * tMat.col1.x + tY * tMat.col1.y);
		var p1Y = (tX * tMat.col2.x + tY * tMat.col2.y);
		
		tX = input.p2.x - transform.position.x;
		tY = input.p2.y - transform.position.y;
		tMat = transform.R;
		var p2X = (tX * tMat.col1.x + tY * tMat.col1.y);
		var p2Y = (tX * tMat.col2.x + tY * tMat.col2.y);
		
		var dX = p2X - p1X;
		var dY = p2Y - p1Y;
		var index = -1;
		
		for (var i = 0; i < this.m_vertexCount; ++i)
		{
			
			
			
			
			
			tVec = this.m_vertices[i];
			tX = tVec.x - p1X;
			tY = tVec.y - p1Y;
			tVec = this.m_normals[i];
			var numerator = (tVec.x*tX + tVec.y*tY);
			
			var denominator = (tVec.x * dX + tVec.y * dY);
			
			if (denominator == 0.0)
			{
				if (numerator < 0.0)
				{
					return false;
				}
			}
			else
			{
				
				
				
				
				if (denominator < 0.0 && numerator < lower * denominator)
				{
					
					
					lower = numerator / denominator;
					index = i;
				}
				else if (denominator > 0.0 && numerator < upper * denominator)
				{
					
					
					upper = numerator / denominator;
				}
			}
			
			if (upper < lower - Number.MIN_VALUE)
			{
				return false;
			}
		}
		
		
		
		if (index >= 0)
		{
			output.fraction = lower;
			
			tMat = transform.R;
			tVec = this.m_normals[index];
			output.normal.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
			output.normal.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
			return true;
		}
		
		return false;
	}
b2PolygonShape.prototype.ComputeAABB = function (aabb, xf) {
		
		var tMat = xf.R;
		var tVec = this.m_vertices[0];
		var lowerX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		var lowerY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		var upperX = lowerX;
		var upperY = lowerY;
		
		for (var i = 1; i < this.m_vertexCount; ++i)
		{
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
	}
b2PolygonShape.prototype.ComputeMass = function (massData, density) {
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		if (this.m_vertexCount == 2)
		{
			massData.center.x = 0.5 * (this.m_vertices[0].x + this.m_vertices[1].x);
			massData.center.y = 0.5 * (this.m_vertices[0].y + this.m_vertices[1].y);
			massData.mass = 0.0;
			massData.I = 0.0;
			return;
		}
		
		
		var centerX = 0.0;
		var centerY = 0.0;
		var area = 0.0;
		var I = 0.0;
		
		
		
		
		var p1X = 0.0;
		var p1Y = 0.0;
		
		
		var k_inv3 = 1.0 / 3.0;
		
		for (var i = 0; i < this.m_vertexCount; ++i)
		{
			
			
			
			
			var p2 = this.m_vertices[i];
			
			var p3 = i + 1 < this.m_vertexCount ? this.m_vertices[parseInt(i+1)] : this.m_vertices[0];
			
			
			var e1X = p2.x - p1X;
			var e1Y = p2.y - p1Y;
			
			var e2X = p3.x - p1X;
			var e2Y = p3.y - p1Y;
			
			
			var D = e1X * e2Y - e1Y * e2X;
			
			
			var triangleArea = 0.5 * D;
			area += triangleArea;
			
			
			
			centerX += triangleArea * k_inv3 * (p1X + p2.x + p3.x);
			centerY += triangleArea * k_inv3 * (p1Y + p2.y + p3.y);
			
			
			var px = p1X;
			var py = p1Y;
			
			var ex1 = e1X;
			var ey1 = e1Y;
			
			var ex2 = e2X;
			var ey2 = e2Y;
			
			
			var intx2 = k_inv3 * (0.25 * (ex1*ex1 + ex2*ex1 + ex2*ex2) + (px*ex1 + px*ex2)) + 0.5*px*px;
			
			var inty2 = k_inv3 * (0.25 * (ey1*ey1 + ey2*ey1 + ey2*ey2) + (py*ey1 + py*ey2)) + 0.5*py*py;
			
			I += D * (intx2 + inty2);
		}
		
		
		massData.mass = density * area;
		
		
		
		
		centerX *= 1.0 / area;
		centerY *= 1.0 / area;
		
		massData.center.Set(centerX, centerY);
		
		
		massData.I = density * I;
	}
b2PolygonShape.prototype.ComputeSubmergedArea = function (
			normal,
			offset,
			xf,
			c) {
		
		var normalL = b2Math.MulTMV(xf.R, normal);
		var offsetL = offset - b2Math.Dot(normal, xf.position);
		
		var depths = new Array();
		var diveCount = 0;
		var intoIndex = -1;
		var outoIndex = -1;
		
		var lastSubmerged = false;
		var i = 0;
		for (i = 0; i < this.m_vertexCount;++i)
		{
			depths[i] = b2Math.Dot(normalL, this.m_vertices[i]) - offsetL;
			var isSubmerged = depths[i] < -Number.MIN_VALUE;
			if (i > 0)
			{
				if (isSubmerged)
				{
					if (!lastSubmerged)
					{
						intoIndex = i - 1;
						diveCount++;
					}
				}
				else
				{
					if (lastSubmerged)
					{
						outoIndex = i - 1;
						diveCount++;
					}
				}
			}
			lastSubmerged = isSubmerged;
		}
		switch(diveCount)
		{
			case 0:
			if (lastSubmerged )
			{
				
				var md = new b2MassData();
				this.ComputeMass(md, 1);
				c.SetV(b2Math.MulX(xf, md.center));
				return md.mass;
			}
			else
			{
				
				return 0;
			}
			break;
			case 1:
			if (intoIndex == -1)
			{
				intoIndex = this.m_vertexCount - 1;
			}
			else
			{
				outoIndex = this.m_vertexCount - 1;
			}
			break;
		}
		var intoIndex2 = (intoIndex + 1) % this.m_vertexCount;
		var outoIndex2 = (outoIndex + 1) % this.m_vertexCount;
		var intoLamdda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex]);
		var outoLamdda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex]);
		
		var intoVec = new b2Vec2(this.m_vertices[intoIndex].x * (1 - intoLamdda) + this.m_vertices[intoIndex2].x * intoLamdda,
										this.m_vertices[intoIndex].y * (1 - intoLamdda) + this.m_vertices[intoIndex2].y * intoLamdda);
		var outoVec = new b2Vec2(this.m_vertices[outoIndex].x * (1 - outoLamdda) + this.m_vertices[outoIndex2].x * outoLamdda,
										this.m_vertices[outoIndex].y * (1 - outoLamdda) + this.m_vertices[outoIndex2].y * outoLamdda);
										
		
		var area = 0;
		var center = new b2Vec2();
		var p2 = this.m_vertices[intoIndex2];
		var p3;
		
		
		i = intoIndex2;
		while (i != outoIndex2)
		{
			i = (i + 1) % this.m_vertexCount;
			if(i == outoIndex2)
				p3 = outoVec
			else
				p3 = this.m_vertices[i];
			
			var triangleArea = 0.5 * ( (p2.x - intoVec.x) * (p3.y - intoVec.y) - (p2.y - intoVec.y) * (p3.x - intoVec.x) );
			area += triangleArea;
			
			center.x += triangleArea * (intoVec.x + p2.x + p3.x) / 3;
			center.y += triangleArea * (intoVec.y + p2.y + p3.y) / 3;
			
			p2 = p3;
		}
		
		
		center.Multiply(1 / area);
		c.SetV(b2Math.MulX(xf, center));
		
		return area;
	}
b2PolygonShape.prototype.GetVertexCount = function () {
		return this.m_vertexCount;
	}
b2PolygonShape.prototype.GetVertices = function () {
		return this.m_vertices;
	}
b2PolygonShape.prototype.GetNormals = function () {
		return this.m_normals;
	}
b2PolygonShape.prototype.GetSupport = function (d) {
		var bestIndex = 0;
		var bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
		for (var i= 1; i < this.m_vertexCount; ++i)
		{
			var value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
			if (value > bestValue)
			{
				bestIndex = i;
				bestValue = value;
			}
		}
		return bestIndex;
	}
b2PolygonShape.prototype.GetSupportVertex = function (d) {
		var bestIndex = 0;
		var bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
		for (var i= 1; i < this.m_vertexCount; ++i)
		{
			var value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
			if (value > bestValue)
			{
				bestIndex = i;
				bestValue = value;
			}
		}
		return this.m_vertices[bestIndex];
	}
// attributes
b2PolygonShape.prototype.m_centroid =  null;
b2PolygonShape.prototype.m_vertices =  null;
b2PolygonShape.prototype.m_normals =  null;
b2PolygonShape.prototype.m_vertexCount =  0;// aliases  
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
// exports  
module.exports =  b2PolygonShape;
