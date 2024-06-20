// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef SESSION_H
#define SESSION_H

#include <imgui/imgui.h>
#include <box2d/box2d.h>
#include "draw.h"
#include <iostream>
#include <vector>
#include <random>
#include <map>
#include <unordered_map>
#include <stdlib.h>
#define _CRT_SECURE_NO_WARNINGS
using namespace std;
struct Settings;
class Session;

#define	RAND_LIMIT 32767

enum UNIT_ID{
	U_ASTRONOMY_ZERO,
	U_ASTRONOMY_FIXED_STAR,
	U_ASTRONOMY_PLANET,
	U_ASTRONOMY_EDGE,
	U_ASTRONOMY_COMET,
	U_ASTRONOMY_SATELIE,
	U_RACE_CYLINDER
};

/// Random number in range [-1,1]
inline float RandomFloat()
{
	float r = (float)(rand() & (RAND_LIMIT));
	r /= RAND_LIMIT;
	r = 2.0f * r - 1.0f;
	return r;
}

/// Random floating point number in range [lo, hi]
inline float RandomFloat(float lo, float hi)
{
	float r = (float)(rand() & (RAND_LIMIT));
	r /= RAND_LIMIT;
	r = (hi - lo) * r + lo;
	return r;
}

// This is called when a joint in the world is implicitly destroyed
// because an attached body is destroyed. This gives us a chance to
// nullify the mouse joint.
class DestructionListener : public b2DestructionListener
{
public:
	void SayGoodbye(b2Fixture* fixture) override { B2_NOT_USED(fixture); }
	void SayGoodbye(b2Joint* joint) override;

	Session* session;
};

const int32 k_maxContactPoints = 2048;

struct ContactPoint
{
	b2Fixture* fixtureA;
	b2Fixture* fixtureB;
	b2Vec2 normal;
	b2Vec2 position;
	b2PointState state;
	float normalImpulse;
	float tangentImpulse;
	float separation;
};

struct s_unit{

};
struct session_object_unit{
	string name;
	int u_id;
	int counts;
	session_object_unit& operator=(const session_object_unit& other) {  
        name = other.name;  
        u_id = other.u_id;
		counts = other.counts;
        return *this;  
    }  
};
struct session_object_param{
	string name;
	string type;
	string value_str;
	int value_int;
	float value_float;
	string unit;
	char value_ch[256];
	session_object_param& operator=(const session_object_param& other) {  
        name = other.name;  
        type = other.type;
		value_str = other.value_str;
		value_int = other.value_int;
		value_float = other.value_float;
		unit = unit;
		strcpy_s(value_ch,other.value_ch);
        return *this;  
    }  
};

struct session_unit_param{
	string name;
	string type;
	string value_str;
	int value_int;
	float value_float;
	string unit;
	char value_ch[256];
	
	session_unit_param& operator=(const session_unit_param& other) {  
        name = other.name;  
        type = other.type;
		value_str = other.value_str;
		value_int = other.value_int;
		value_float = other.value_float;
		unit = unit;
		strcpy_s(value_ch,other.value_ch);
        return *this;  
    }  
};
struct session_unit_property{
	vector<session_unit_param> unit_property;
};


class Session : public b2ContactListener
{
public:

	Session();
	virtual ~Session();

	virtual void CleanSession();
	void DrawTitle(const char* string);
	virtual void Step(Settings& settings);
	virtual void UpdateUI() {}
	virtual void Keyboard(int key) { B2_NOT_USED(key); }
	virtual void KeyboardUp(int key) { B2_NOT_USED(key); }
	void ShiftMouseDown(const b2Vec2& p);
	virtual void MouseDown(const b2Vec2& p);
	virtual void MouseUp(const b2Vec2& p);
	virtual void MouseDownRight(const b2Vec2& p);
	virtual void MouseUpRight(const b2Vec2& p);
	virtual void MouseMove(const b2Vec2& p);
	void LaunchBomb();
	void LaunchBomb(const b2Vec2& position, const b2Vec2& velocity);
	
	void SpawnBomb(const b2Vec2& worldPt);
	void CompleteBombSpawn(const b2Vec2& p);

	virtual void CreateSelectedUnit(const b2Vec2& p);

	// Let derived tests know that a joint was destroyed.
	virtual void JointDestroyed(b2Joint* joint) { B2_NOT_USED(joint); }

	// Callbacks for derived classes.
	virtual void BeginContact(b2Contact* contact)  override { B2_NOT_USED(contact); }
	virtual void EndContact(b2Contact* contact)  override { B2_NOT_USED(contact); }
	virtual void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) override;
	virtual void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) override
	{
		B2_NOT_USED(contact);
		B2_NOT_USED(impulse);
	}
	
	void ShiftOrigin(const b2Vec2& newOrigin);
	void InitialUnitParam();
	void UpdateUnitParam(int unit_id);
	vector<session_object_unit> s_o_unit;
	vector<session_unit_property> all_unit_pro;
	

	vector<session_object_param> s_o_param;
	int selected_unit_index=0;
	int creating_unit_index=0;
	bool if_mouse_right_down_pause =false;
	b2Vec2 mouse_pos;

	bool if_draw_trails = false;
	int trails_len = 100;
	virtual void UpdateTrails();
	virtual void DrawTrails();

	vector<session_unit_param> unit_property_mouse_rdown;
	bool  if_mouse_right_down_select_body = false;
	bool  if_mouse_right_down_delete_body = false;
	virtual void SetRightSelectBody();
	virtual void DelRightSelectBody();
	b2Body* m_mouse_right_down=nullptr;
	// sesion id, 1 for astronomy
	int session_id;
	virtual void DeleteBodys();

	bool if_show_unit_name = false;
	virtual void show_unit_name();

	virtual void Start_Race();
private:
	
	

protected:
	friend class DestructionListener;
	friend class BoundaryListener;
	friend class ContactListener;

	b2Body* m_groundBody;
	b2AABB m_worldAABB;
	ContactPoint m_points[k_maxContactPoints];
	int32 m_pointCount;
	DestructionListener m_destructionListener;
	int32 m_textLine;
	b2World* m_world;
	b2Body* m_bomb;
	b2MouseJoint* m_mouseJoint;
	b2Vec2 m_bombSpawnPoint;
	bool m_bombSpawning;
	b2Vec2 m_mouseWorld;
	int32 m_stepCount;
	int32 m_textIncrement;
	b2Profile m_maxProfile;
	b2Profile m_totalProfile;

	vector<session_object_unit> unit_all={
		{"zero",U_ASTRONOMY_ZERO,0},
		{"fixed star",U_ASTRONOMY_FIXED_STAR,0},
		{"planet",U_ASTRONOMY_PLANET,0},
		{"edge",U_ASTRONOMY_EDGE,0},
		{"comet",U_ASTRONOMY_COMET,0},
		{"satelite",U_ASTRONOMY_SATELIE,0},
		{"cylinder",U_RACE_CYLINDER,0}
	};
	
	unordered_map<b2Body*, vector<b2Vec2>> trails;

	
	vector<b2Body*> delete_bodys;
};

typedef Session* SessionCreateFcn();

int RegisterSession(const char* category, const char* name, SessionCreateFcn* fcn);

//
struct SessionEntry
{
	const char* category;
	const char* name;
	SessionCreateFcn* createFcn;
};

#define MAX_SESSIONS 256
extern SessionEntry g_testEntries[MAX_SESSIONS];
extern int g_testCount;

#endif
