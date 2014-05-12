#include "debugDraw.h"

/// Draw a closed polygon provided in CCW order.
void debugDraw::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) {
	//ci::app::console() << "TESTDRAW_POLYGON" << std::endl;
	ci::gl::color( ci::Color(color.r, color.g, color.b) );
	ci::gl::begin(GL_LINE_LOOP);
	for(int i = 0; i < vertexCount; i++){
		ci::gl::vertex(vertices[i].x,vertices[i].y);
	}
	ci::gl::end();
}

/// Draw a solid closed polygon provided in CCW order.
void debugDraw::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) {
	//ci::app::console() << "TESTDRAW_SOLID_POLYGON" << std::endl;
	ci::gl::color( ci::Color(color.r, color.g, color.b) );
	ci::gl::begin(GL_POLYGON);
	for(int i = 0; i < vertexCount; i++){
		ci::gl::vertex(vertices[i].x,vertices[i].y);
	}
	ci::gl::end();
}

/// Draw a circle.
void debugDraw::DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color) {
	//ci::app::console() << "TESTDRAW_CIRCLE" << std::endl;
	ci::gl::color( ci::Color(color.r, color.g, color.b) );
	ci::gl::drawStrokedCircle(ci::Vec2f(center.x,center.y), radius);
}

/// Draw a solid circle.
void debugDraw::DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color) {
	//ci::app::console() << "TESTDRAW_SOLID_CIRCLE" << std::endl;
	ci::gl::color( ci::Color(color.r, color.g, color.b) );
	ci::gl::drawSolidCircle(ci::Vec2f(center.x,center.y), radius);
}

/// Draw a line segment.
void debugDraw::DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) {
	//ci::app::console() << "TESTDRAW_LINE_SEGMENT" << std::endl;
	ci::gl::color( ci::Color(color.r, color.g, color.b) );
	ci::gl::drawLine( ci::Vec3f( p1.x, p1.y, 0.0) , ci::Vec3f( p2.x, p2.y, 0.0) );
}

/// Draw a transform. Choose your own length scale.
/// @param xf a transform.
void debugDraw::DrawTransform(const b2Transform& xf) {
	ci::app::console() << "TESTDRAW_TRANSFORM" << std::endl;
	
}


void DrawPoint(const b2Vec2& p, float32 size, const b2Color& color){
	ci::gl::color( ci::Color(color.r, color.g, color.b) );
	ci::gl::drawSolidCircle(ci::Vec2f(p.x, p.y), size);
}

void DrawString(int x, int y, const char* string, ...){
	ci::gl::drawString(string, ci::Vec2f( (float32)x, (float32)y ) );
}

void DrawString(const b2Vec2& p, const char* string, ...){
	ci::gl::drawString(string, ci::Vec2f( p.x, p.y ) );
}

void DrawAABB(b2AABB* aabb, const b2Color& color){
	ci::gl::color( ci::Color(color.r, color.g, color.b) );
}