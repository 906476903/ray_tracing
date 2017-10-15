#ifndef RENDER_H
#define RENDER_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <cstdio>
#include <cstring>
#include <fstream>
#include "constant.h"
#include "tools/rand_mat.h"

class Vec3
{
public:
  union
	{
		struct {float x, y, z;};
		struct {float r, g, b;};
		struct {float gr[3];};
	};
	Vec3():x(0), y(0), z(0){}
	Vec3(float x, float y, float z):x(x),y(y),z(z){}
	void Normalize() { real l = 1.0f / Length(); x *= l; y *= l; z *= l; }
	real Length() { return (real)sqrt( x * x + y * y + z * z ); }
	real Dot( Vec3 a_V ) { return x * a_V.x + y * a_V.y + z * a_V.z; }
	Vec3 Cross( Vec3 b ) { return Vec3( y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x ); }
	Vec3 operator - () const{return Vec3(-x, -y, -z);}
	void operator += (Vec3& tmp) {x += tmp.x; y += tmp.y; z += tmp.z;}
	void operator *= (float f) {x *= f; y *= f; z *= f;}
	friend Vec3 operator + (const Vec3& l, const Vec3& r) {return Vec3(l.x + r.x, l.y + r.y, l.z + r.z);}
	friend Vec3 operator - (const Vec3& l, const Vec3& r) {return Vec3(l.x - r.x, l.y - r.y, l.z - r.z);}
	friend Vec3 operator * (const Vec3& l, float f) {return Vec3(l.x * f, l.y * f, l.z * f);}
	friend Vec3 operator * (const Vec3& l, Vec3& r) {return Vec3(l.x * r.x, l.y * r.y, l.z * r.z);}
	friend Vec3 operator * (float f, const Vec3& r) {return r*f;}
	
};
typedef Vec3 Color;

struct Ray{
  Vec3 from_;
	Vec3 dir_;
	Ray():from_(Vec3(0,0,0)), dir_(Vec3(0,0,0)){};
  Ray(Vec3& f_, Vec3& d_):from_(f_), dir_(d_){}

};

inline real sqr(real r1) {return r1 * r1;}
struct Texture {
  int W, H, area;
  Color* color_;
	
	Texture(){}
	Texture(char* File_) {
    unsigned char b[1000000];
    FILE* f1 = fopen(File_, "rb");
    fread(b, 1, 20, f1);
    W = (1<<8) * b[13] + b[12];
    H = (1<<8) * b[15] + b[14];
    area = W * H;
    color_ = new Color[area];
    FILE* f2 = fopen(File_, "rb");
    fread(b, 1, W*H*3+30, f2);
    for(int i = 0; i < area; ++i) {
      int j = i * 3;
      float k = (1<<8);
      color_[i] = Color(1.0*b[j+20]/k, 1.0*b[j+19]/k, 1.0*b[j+18]/k);
    }
    fclose(f1);fclose(f2);
  }
	Color Get_color(float r, float l){
    float u = (r + 1000.5f) * W;
    float v = (l + 1000.0f) * W;
    int u1 = ((int)u) % W, v1 = ((int)v) % H;
    int u2 = (((int)u) % W + 1) % W, v2 = (((int)v) % H + 1) % H;
    float f_u = u - floorf(u), _f_u = 1 - f_u, f_v = v - floorf(v), _f_v = 1 - f_v;
    return color_[u1 + v1 * W] * _f_u * _f_v + color_[u2 + v1 * W] * f_u * _f_v + color_[u1 + v2 * W] * _f_u * f_v + color_[u2 + v2 * W] * f_u *  f_v;
  }
};

struct Attr {
  Color color_;
	real reflec_, refrac_, diff_, spec_, d_reflec, ind_, u_scl, v_scl;
	Texture* texture_;
  
	Attr(){
    color_ = Color(0.2f, 0.2f, 0.2f);
    reflec_ = refrac_ = d_reflec = 0;
    diff_ = 0.2f;
    spec_ = 0.8f;
    ind_ = 1.5f;
    u_scl = v_scl = 1.0f;
    texture_ = NULL;
  }
  
	void SetPrams(real _reflec_, real _refrac_, Color& _colo_, real _diff_, real _spec_){
    reflec_ = _reflec_;
    refrac_ = _refrac_;
    color_ = _colo_;
    diff_ = _diff_;
    spec_ = _spec_;
  }

	void SetRefrInd(real u) {ind_ = u;}
	void SetD_Refl(real u) {d_reflec = u;}
  void SetTexture(Texture* pr) {texture_ = pr;}
  
	void SetUVScale(real uscl_, real vscl_) {u_scl = uscl_; v_scl = vscl_;}
};

struct Vex {
  Vec3 pos_, N_;
	real u_, v_;
	Vex() {};
	Vex(Vec3 p, real r1, real r2):pos_(p), u_(r1), v_(r2){};
	Vex(real v1, real v2, real v3){pos_ = Vec3(v1, v2, v3);};
	Vec3& GetPos() { return pos_; }//
	void SetUV(real r1, real r2){u_ = r1; v_ = r2;}
};
struct aabb {
  Vec3 pos_, _size, end_;
	aabb():pos_(Vec3(0, 0, 0)), _size(Vec3(0, 0, 0)), end_(Vec3(0, 0, 0)){};
	aabb(Vec3& r1, Vec3& r2):pos_(r1), _size(r2) {end_ = r1 + r2;};
	bool Contains(Vec3 tmp){
		return ((tmp.x >= pos_.x) && (tmp.x <= end_.x) && (tmp.y >= pos_.y) && (tmp.y <= end_.y) && (tmp.z >= pos_.z) && (tmp.z <= end_.z));
	}

};

class Abs_obj {
public:
  Attr* mal_;
  Vec3 pos_, vn, vc, N_;
  real _r, u_, v_, nu, nv, nd, bnu, bnv, cnu, cnv;
	int k, _id;
  Vex* tri_pos[3];
  
	Abs_obj(int id, Vec3& pos, real r){
    pos_ = pos; _r = r;
    _id = id;
    mal_ = new Attr();
    vn = Vec3(0, 1, 0);
    vc = vn.Cross(Vec3(1, 0, 0));
  }
	Abs_obj(int id, Vex* v1, Vex* v2, Vex* v3):_id(id), mal_(NULL), k(2) {
    tri_pos[0] = v1, tri_pos[1] = v2, tri_pos[2] = v3;
    Vec3 l = tri_pos[1]->GetPos() - tri_pos[0]->GetPos(), r = tri_pos[2]->GetPos() - tri_pos[0]->GetPos();
    N_ = r.Cross(l);
    if(fabsf(N_.x) > fabsf(N_.y) && fabsf(N_.x) > fabsf(N_.z)) k = 0;
    if(fabsf(N_.x) <= fabsf(N_.y) && fabsf(N_.z) <= fabsf(N_.y)) k = 1;
    int u = (k + 1) % 3, v = (k + 2) % 3;
    nu = 1.0 * N_.gr[u] / N_.gr[k];
    nv = 1.0 * N_.gr[v] / N_.gr[k];
    nd = 1.0 * N_.Dot(tri_pos[0]->GetPos()) / N_.gr[k];
    real e = r.gr[u] * l.gr[v] - r.gr[v] * l.gr[u];
    bnu = r.gr[u] / e, bnv = -r.gr[v] / e, cnu = l.gr[v] / e, cnv = -l.gr[u] / e;
    N_.Normalize();
    tri_pos[0]->N_ = tri_pos[1]->N_ = tri_pos[2]->N_ = N_;
  }
	~Abs_obj(){if (_id == 1) delete mal_;}
	int Intersect(Ray& r, real& dist){
  
    if (_id == 1){
      Vec3 v = r.from_ - pos_;
      real sq = -DOT(v, r.dir_);
      real det = (sq * sq) - DOT(v, v) + sqr(_r);
      if(det < 0) return MISS;
      real u_ = sq - sqrtf(det), v_ = sq + sqrtf(det );
      if(v_ <= 0) return MISS;
      if(v_ < dist && u_ < 0) {dist = v_; return INPRIM;}
      if(u_ < dist && u_ >= 0) {dist = u_; return HIT;}
      return MISS;
    }
    else {
      int r1, r2;
      if(k == 0) {r1 = 1, r2 = 2;}
      if(k == 1) {r1 = 2, r2 = 0;}
      if(k == 2) {r1 = 0, r2 = 1;}
      Vec3 tmp_pos = r.from_, tmp_dir = r.dir_;
      real tp = 1.0 * (nd - tmp_pos.gr[k] - nu * tmp_pos.gr[r1] - nv * tmp_pos.gr[r2]) / (tmp_dir.gr[k] + nu * tmp_dir.gr[r1] + nv * tmp_dir.gr[r2]);
      if (!(tp > 0 && dist > tp)) return MISS;
      real hu = tmp_pos.gr[r1] + tp * tmp_dir.gr[r1] - tri_pos[0]->GetPos().gr[r1];
      real hv = tmp_pos.gr[r2] + tp * tmp_dir.gr[r2] - tri_pos[0]->GetPos().gr[r2];
      real beta = u_ = hv * bnu + hu * bnv, gamma = v_ = hu * cnu + hv * cnv;
      if (beta < 0 || gamma < 0 || (u_ + v_) > 1) return MISS;
      dist = tp;
      if(DOT( tmp_dir, N_ ) > 0) return INPRIM;
      return HIT;
    }
  }
	bool Inter_aabb(aabb& _g) {
    if (_id == 1) return Inter_sphere(pos_, _g);
    else return Inter_tri(_g.pos_ + 0.5 * _g._size, 0.5 * _g._size, tri_pos[0]->pos_, tri_pos[1]->pos_, tri_pos[2]->pos_);
  }
	void cal_range(real& l_, real& r_, int d) {
    if (_id == 1) {
      l_ = pos_.gr[d] - _r;
      r_ = pos_.gr[d] + _r;
    }
    else {
      l_ = tri_pos[0]->GetPos().gr[d], r_ = tri_pos[0]->GetPos().gr[d];
      for (int i = 1; i < 3; i++){
        l_ = std::min(l_, tri_pos[i]->GetPos().gr[d]);
        r_ = std::max(r_, tri_pos[i]->GetPos().gr[d]);
      }
    }
  }
	Vec3 Norm_v(Vec3& v) {
    if(_id == 1) return (v - pos_) * (1.0 / _r); 
    else {
      Vec3 N = tri_pos[0]->N_ + u_ * (tri_pos[1]->N_ - tri_pos[0]->N_) + v_ * (tri_pos[2]->N_ - tri_pos[0]->N_);
      N.Normalize();
      return N;
    }
  }
	Color Get_color(Vec3& _p) {
	if(!mal_->texture_) return mal_->color_;
    if(_id == 1) {
      Vec3 vp = (_p - pos_) * (1.0 / _r);
      real phi = acosf(-DOT(vp, vn));
      real theta = (2.0f * acosf(DOT(Vec3(1, 0, 0), vp) / sinf(phi))) / PI;
      real u = theta / mal_->u_scl, v = phi / mal_->v_scl / PI;
      if(DOT(vc, vp) >= 0) u = (1.0f - theta) / mal_->u_scl;
      return mal_->texture_->Get_color(u, v) * mal_->color_;
    }
    else{
      real U1 = tri_pos[0]->u_, V1 = tri_pos[0]->v_;
      real U2 = tri_pos[1]->u_ - U1, V2 = tri_pos[1]->v_ - V1;		
      real U3 = tri_pos[2]->u_ - U1, V3 = tri_pos[2]->v_ - V1;		
      real u = U1 + u_ * U2 + v_ * U3, v = V1 + u_ * V2 + v_ * V3;
      return mal_->texture_->Get_color(u, v) * mal_->color_;
    }
  }
	

  bool is_ins_plane(Vec3& v1, Vec3& v2, Vec3& v3) {
    Vec3 _min, _max;
    for(int j = 0; j < 3; j++)
      if (v1.gr[j] > 0.0f){
        _min.gr[j] = -v3.gr[j] - v2.gr[j];
        _max.gr[j] =  v3.gr[j] - v2.gr[j];
      }
      else{
        _min.gr[j] =  v3.gr[j] - v2.gr[j];
        _max.gr[j] = -v3.gr[j] - v2.gr[j];
      }
    if (DOT( v1, _min) > 0.0f) return false;
    if (DOT( v1, _max) >= 0.0f) return true;
    return false;
  }

  bool Inter_tri(Vec3& b_pos, Vec3& hsize_, Vec3& p0_, Vec3& p1_, Vec3& p2_) {
    float min, max, p0, p1, p4, rad, fex, fey, fez;
    Vec3 v0 = p0_ - b_pos;
    float v00 = v0.gr[0], v01 = v0.gr[1], v02 = v0.gr[2];
    Vec3 v1 = p1_ - b_pos;
    float v10 = v1.gr[0], v11 = v1.gr[1], v12 = v1.gr[2];
    Vec3 v2 = p2_ - b_pos;
    float v20 = v2.gr[0], v21 = v2.gr[1], v22 = v2.gr[2];
    Vec3 e0 = v1 - v0;
    float e00 = e00 = e0.gr[0], e01 = e0.gr[1], e02 = e0.gr[2];
    Vec3 e1 = v2 - v1;
    float e10 = e1.gr[0], e11 = e1.gr[1], e12 = e1.gr[2];
    Vec3 e2 = v0 - v2;
    float e20 = e2.gr[0], e21 = e2.gr[1], e22 = e2.gr[2];
    p0 = e02 * v01 - e01 * v02, p4 = e02 * v21 - e01 * v22;
    rad = fabsf(e02) * hsize_.gr[1] + fabsf(e01) * hsize_.gr[2];
    if (std::min(p0, p4) > rad || std::max(p0, p4) < -rad) return 0;
    p0 = -e02 * v00 + e00 * v02, p4 = -e02 * v20 + e00 * v22;
    rad = fabsf(e02) * hsize_.gr[0] + fabsf(e00) * hsize_.gr[2];
    if (std::min(p0, p4) > rad || std::max(p0, p4) < -rad) return 0;
    p1 = e01 * v10 - e00 * v11, p4 = e01 * v20 - e00 * v21;
    rad = fabsf(e01) * hsize_.gr[0] + fabsf(e00) * hsize_.gr[1];
    if (std::min(p1, p4) > rad || std::max(p1, p4) < -rad) return 0;
    p0 = e12 * v01 - e11 * v02, p4 = e12 * v21 - e11 * v22;
    rad = fabsf(e12) * hsize_.gr[1] + fabsf(e11) * hsize_.gr[2];
    if (std::min(p0, p4) > rad || std::max(p0, p4) < -rad) return 0;
    p0 = -e12 * v00 + e10 * v02, p4 = -e12 * v20 + e10 * v22;
    rad = fabsf(e12) * hsize_.gr[0] + fabsf(e10) * hsize_.gr[2];
    if (std::min(p0, p4) > rad || std::max(p0, p4) < -rad) return 0;
    p0 = e11 * v00 - e10 * v01, p1 = e11 * v10 - e10 * v11;
    rad = fabsf(e11) * hsize_.gr[0] + fabsf(e10) * hsize_.gr[1];
    if (std::min(p0, p1) > rad || std::max(p0, p1) < -rad) return 0;
    p0 = e22 * v01 - e21 * v02, p1 = e22 * v11 - e21 * v12;
    rad = fabsf(e22) * hsize_.gr[1] + fabsf(e21) * hsize_.gr[2];
    if (std::min(p0, p1) > rad || std::max(p0, p1) < -rad) return 0;
    p0 = -e22 * v00 + e20 * v02, p1 = -e22 * v10 + e20 * v12;
    rad = fabsf(e22) * hsize_.gr[0] + fabsf(e20) * hsize_.gr[2];
    if (std::min(p0, p1) > rad || std::max(p0, p1) < -rad) return 0;
    p1 = e21 * v10 - e20 * v11, p4 = e21 * v20 - e20 * v21;
    rad = fabsf(e21) * hsize_.gr[0] + fabsf(e20) * hsize_.gr[1];
    if (std::min(p1, p4) > rad || std::max(p1, p4) < -rad) return 0;
    if (std::min(std::min(v00, v10), v20) > hsize_.gr[0] || std::max(std::max(v00, v10), v20) < -hsize_.gr[0]) return false;
    if (std::min(std::min(v01, v11), v21) > hsize_.gr[1] || std::max(std::max(v01, v11), v21) < -hsize_.gr[1]) return false;
    if (std::min(std::min(v02, v12), v22) > hsize_.gr[2] || std::max(std::max(v02, v12), v22) < -hsize_.gr[2]) return false;
    Vec3 N0 = e0.Cross(e1);
    if (!is_ins_plane(N0, v0, hsize_ )) return false;
    return true;
  }
	bool Inter_sphere(Vec3& _ps, aabb& _g){
    float tmp = 0;
    for(int i = 0; i < 3; i++){
      if (_ps.gr[i] < _g.pos_.gr[i]) tmp += (_ps.gr[i] - _g.pos_.gr[i]) * (_ps.gr[i] - _g.pos_.gr[i]);
      else if (_ps.gr[i] > (_g.end_.gr[i])) tmp += (_ps.gr[i] - (_g.end_.gr[i])) * (_ps.gr[i] - (_g.end_.gr[i]));
    }
    if(tmp > sqr(_r)) return false;
    return true;
  }
};
struct Light {
  int _id;
  Vec3 pos_, d_x, d_y;
	Color color_;
	Vec3* gr;
  
	Light(int id, Vec3& p_, Color& c_):_id(id), pos_(p_), color_(c_), gr(NULL){};
	Light(int id, Vec3& _p1, Vec3& _p2, Vec3& _p3, Color& _c):_id(id), color_(_c) {
    gr = new Vec3[16];//
    gr[ 0] = Vec3( 1, 2, 0 );gr[ 1] = Vec3( 3, 3, 0 );
    gr[ 2] = Vec3( 2, 0, 0 );gr[ 3] = Vec3( 0, 1, 0 );
    gr[ 4] = Vec3( 2, 3, 0 );gr[ 5] = Vec3( 0, 3, 0 );
    gr[ 6] = Vec3( 0, 0, 0 );gr[ 7] = Vec3( 2, 2, 0 );
    gr[ 8] = Vec3( 3, 1, 0 );gr[ 9] = Vec3( 1, 3, 0 );
    gr[10] = Vec3( 1, 0, 0 );gr[11] = Vec3( 3, 2, 0 );
    gr[12] = Vec3( 2, 1, 0 );gr[13] = Vec3( 3, 0, 0 );
    gr[14] = Vec3( 1, 1, 0 );gr[15] = Vec3( 0, 2, 0 );
    d_x = (_p2 - _p1) * 0.25;
    d_y = (_p3 - _p1) * 0.25;
    for(int i = 0; i < 16; i++) gr[i] = gr[i].gr[0] * d_x + gr[i].gr[1] * d_y + _p1;
    pos_ = _p1 + 2 * d_x + 2 * d_y;
  }
	Vec3& GetPos() { return pos_; }//
};

struct Obj_list {
  Abs_obj* chara;
	Obj_list* N_ext;
	Obj_list():chara(NULL), N_ext(NULL) {}
	~Obj_list() { delete N_ext; }
	void Next( Obj_list* a_Next ) { N_ext = a_Next; }
};

#define xFc 0xfffffffc
#define xF8 0xfffffff8
#define xFb 0xfffffffb
#define xFf 0xffffffff
class Node {//speed up
public:
  real dpos;
	unsigned long ofs;
	Node():ofs(6) {};
	void Set_axs( int a_Axis ) { ofs = (ofs & xFc) + a_Axis; }
	Node* GetLeft() { return (Node*)(ofs&xF8); }
	Node* GetRight() { return ((Node*)(ofs&xF8)) + 1; }
	void Add( Abs_obj* a_Prim );
	bool IsLeaf() { return ((ofs & 4) > 0); }
	void SetLeaf( bool a_Leaf ) { ofs = (a_Leaf)?(ofs|4):(ofs&xFb); }
	Obj_list* GetList() { return (Obj_list*)(ofs&xF8); }
	void SetList( Obj_list* a_List ) { ofs = (unsigned long)a_List + (ofs & 7); }
};

class Creater//
{
public:
  Obj_list* objl_, *pr;
	char* kd_mem, *obj_mem;
	Node* pr1;
	Creater() : objl_( 0 ){
    kd_mem = (char*)(new Node[1000000]);
    obj_mem = (char*)(new Obj_list[100000]);
    unsigned long addr = (unsigned long)kd_mem;
    pr1 = (Node*)((addr + 32) & (xFf - 31));
    addr = (unsigned long)obj_mem;
    pr = (Obj_list*)((addr + 32) & (xFf - 31));
    Obj_list* ptr = pr;
    for(int i = 0; i < 99995; i++) {
      ptr->Next( ptr + 1 );
      ptr++;
    }
    ptr->Next(NULL);
    objl_ = pr;
  }
	Obj_list* NewObj_list() {
    Obj_list* ans;
    ans = objl_;
    objl_ = objl_->N_ext;
    ans->Next(NULL);
    ans->chara = NULL;
    return ans;
  }
	void FreeObj_list(Obj_list* dst){
    Obj_list* list = dst;
    while (list->N_ext) list = list->N_ext;
    list->Next(objl_);
    objl_ = dst;
  }
	Node* NewNodePair(){
    ((unsigned long*)pr1)[1] = ((unsigned long*)pr1)[3] = 6;
    Node* node = pr1;
    pr1 += 2;
    return node;
  }
};

struct kdstack {
	Node* node;
	real t;
	Vec3 pb;
	int prev;
};
struct D_list {
	real pos_;
	int tot_l, tot_r;
	D_list* next;
};
class Space;
class KdTree
{
public:
  Node* _root;
	D_list* _slist, *_spool;
  static Creater* s_Creater;
	KdTree(){_root = new Node();}
	KdTree(Space* _objs){_root = new Node(); Build_tree(_objs);}
	void Build_tree(Space* sp);
	void Insert_(real _sp);
	void KdTree::Down_(Node* cur, aabb& gr_, int cur_depth, int size_){
    std::vector<float> el(size_), er(size_);
    std::vector<bool> pos_r(size_);
    aabb n1, n2, n3, n4;
    if (_slist){
      D_list* list = _slist;
      while (list->next) list = list->next; list->next = _spool;
      _spool = _slist;
      _slist = NULL;
    }
    Vec3 s = gr_._size;
    if (s.x >= s.y && s.x >= s.z) cur->Set_axs(0);
    else if (s.x <= s.y && s.y >= s.z) cur->Set_axs(1);
    int axis = cur->ofs & 3;
    real _p1 = gr_.pos_.gr[axis], _p2 = gr_.end_.gr[axis];
    Abs_obj** pr_objs = new Abs_obj*[size_];
    int tot(0);
    Obj_list* l = cur->GetList();
    real p1, p2;
    
    while (l){
      Abs_obj* p = pr_objs[tot] = l->chara;
      pos_r[tot] = true;
      p->cal_range( el[tot], er[tot], axis );
      tot++;
      if (p->_id == 1){
        p1 = p->pos_.gr[axis] - p->_r;
        p2 = p->pos_.gr[axis] + p->_r;
        if ((p1 >= _p1) && (p1 <= _p2)) Insert_( p1 );
        if ((p2 >= _p1) && (p2 <= _p2)) Insert_( p2 );
      }
      else{
        for ( int i = 0; i < 3; i++ ){
          p1 = p->tri_pos[i]->GetPos().gr[axis];
          if ((p1 >= _p1) && (p1 <= _p2)) Insert_( p1 );
        }
      }
      l = l->N_ext;
    }
    n3 = n4 = gr_;
    D_list* cur_pos = _slist;
    float tmp1 = n3.pos_.gr[axis], tmp2 = n4.end_.gr[axis];
    while (cur_pos) {
      n4.pos_.gr[axis] = cur_pos->pos_;
      n4._size.gr[axis] = _p2 - cur_pos->pos_;
      n3._size.gr[axis] = cur_pos->pos_ - _p1;
      for ( int i = 0; i < size_; i++ ) if (pos_r[i]) {
        Abs_obj* p = pr_objs[i];
        if ((el[i] <= n3.end_.gr[axis]) && (er[i] >= tmp1) && p->Inter_aabb( n3 )) cur_pos->tot_l++;
        if ((el[i] <= tmp2) && (er[i] >= n4.pos_.gr[axis]) && p->Inter_aabb( n4 )) cur_pos->tot_r++; else pos_r[i] = false;
      }
      else cur_pos->tot_l++;
      cur_pos = cur_pos->next;
    }
    cur_pos = _slist;
    real min = INF, max = 0;
    while (cur_pos){
      n4.pos_.gr[axis] = cur_pos->pos_;
      n4._size.gr[axis] = _p2 - cur_pos->pos_;
      n3._size.gr[axis] = cur_pos->pos_ - _p1;
      real tp1 = 2 * (n3._size.x * n3._size.z + n3._size.x * n3._size.y + n3._size.z * n3._size.y);
      real tp2 = 2 * (n4._size.x * n4._size.z + n4._size.x * n4._size.y + n4._size.z * n4._size.y);
      real cost_ = 0.3f + 1.0f * 0.5f / (gr_._size.x * gr_._size.z + gr_._size.x * gr_._size.y + gr_._size.z * gr_._size.y) * (tp1 * cur_pos->tot_l + tp2 * cur_pos->tot_r);
      if (cost_ < min){
        min = cost_;
        max = cur_pos->pos_;
        n1 = n3, n2 = n4;
      }
      cur_pos = cur_pos->next;
    }
    if (min > size_) return;
    cur->dpos = max;
    Node* left = s_Creater->NewNodePair();
    int tot_l = 0, tot_r = 0, total = 0;
    for(int i = 0; i < size_; i++ ){
      Abs_obj* p = pr_objs[i];
      total++;
      if ((el[i] <= n1.end_.gr[axis]) && (er[i] >= n1.pos_.gr[axis])) if (p->Inter_aabb( n1 )) {
        left->Add( p );
        tot_l++;
      }
      if ((el[i] <= n2.end_.gr[axis]) && (er[i] >= n2.pos_.gr[axis])) if (p->Inter_aabb( n2 )) {
        (left + 1)->Add( p );
        tot_r++;
      }
    }
    delete pr_objs;
    s_Creater->FreeObj_list(cur->GetList());
    cur->ofs = (unsigned long)left + (cur->ofs & 7);
    cur->SetLeaf(false);
    if(cur_depth < MAXTREEDEPTH){
      if (tot_l > 2) Down_( left, n1, cur_depth + 1, tot_l );
      if (tot_r > 2) Down_( left + 1, n2, cur_depth + 1, tot_r );
    }
  }
	static void Init(Creater* pr) {s_Creater = pr; }
};


class Space{
public:
	Abs_obj** chara;
	Light** Lig;
	aabb Frw;
	KdTree* tree;
  float* _Verts, *_TCoords;
	int charas, num_lig, num_V, num_F;
	unsigned short* _Faces;
	Space():charas(NULL), chara(NULL),  Frw(Vec3(0,0,0), Vec3(0,0,0)){}
	~Space(){delete chara;}
	void Space::InitSpace(std::vector<real> &w1, std::vector<real> &w2, std::vector<real> &w3, std::vector<int> &j1, std::vector<int> &j2, std::vector<int> &j3);
	void Space::Mk_home();
  void Space::Mk_light();
  void Space::Mk_plane(Vec3 a_P1, Vec3 a_P2, Vec3 a_P3, Vec3 a_P4, Attr* a_Mat);
  void Space::load_from_obj(Attr* a_Attr, real a_Size, std::vector<real> &w1, std::vector<real> &w2, std::vector<real> &w3, std::vector<int> &j1, std::vector<int> &j2, std::vector<int> &j3);
};

class Render {
public:
  unsigned int* red_, *green_, *blue_;
	int W, H, area;
	Vec3 from_, _P1, _P2, _P3, _P4, m_DX, m_DY;//
	int* _Mod;
	kdstack* stk;
  Space* objs;
  std::vector<real> w1, w2, w3;
  std::vector<int> j1, j2, j3;

	Render() {//
    char buf;
    std::ifstream infile("meshes/cube.obj");
    while (1) {
      infile >> buf;
      if (buf == 'v') {
        double x, y, z;
        infile >> x>>y>>z;
        w1.push_back(x);
        w2.push_back(y);
        w3.push_back(z);
      } else if (buf == 'f') {
        int x, y, z;
        infile >> x>>y>>z;
        j1.push_back(x);
        j2.push_back(y);
        j3.push_back(z);
      }
      else if (buf == '#') break;
    }
    infile.close();
    std::cout<<"prepare for rendering";
    objs = new Space();
    _Mod = new int[64];
    _Mod = (int*)((((unsigned long)_Mod) + 32) & (0xffffffff - 31));
    _Mod[0] = 0, _Mod[1] = 1, _Mod[2] = 2, _Mod[3] = 0, _Mod[4] = 1;
    stk = new kdstack[64];
    stk = (kdstack*)((((unsigned long)stk) + 32) & (0xffffffff - 31));
    KdTree::Init(new Creater());
  }
  ~Render(){delete objs;}
	void Prepare_data(int w_, int h_){
    W = w_;H = h_;
    area = W * H;
    red_ = new unsigned int[area];
    green_ = new unsigned int[area];
    blue_ = new unsigned int[area];
    objs->InitSpace(w1, w2, w3, j1, j2, j3);
    from_ = Vec3(-4.88675, 4.88675, -4.88675);
    _P1 = Vec3(-3.3, 4.0, 1.3);
    _P2 = Vec3(1.3, 4.0, -3.3);
    _P3 = Vec3(-0.69, 0, -5.3);
    _P4 = Vec3(-5.3, 0, -0.69);
    m_DX = (_P2 - _P1) * (1.0f / W);
    m_DY = (_P4 - _P1) * (1.0f / H);
  }
  Twister R_mat;
	int Find_obj_s(Ray& r_, real& _dist, Abs_obj*& N_obj ) {
	Vec3 p1 = objs->Frw.pos_;
	Vec3 p4 = p1 + objs->Frw._size;
	Vec3 D = r_.dir_, O = r_.from_;
	for(int i = 0; i < 3; ++i)
  if (D.gr[i] < 0) {
		if (O.gr[i] < p1.gr[i]) return 0;
	}
	else if (O.gr[i] > p4.gr[i]) return 0;
  real d_near = 0, d_far = _dist;
	for (int i = 0; i < 3; i++ ){
		real pos = O.gr[i] + d_far * D.gr[i];
		if (D.gr[i] < 0) {
			if (pos < p1.gr[i]) d_far = d_near + (d_far - d_near) * ((O.gr[i] - p1.gr[i]) / (O.gr[i] - pos));
			if (O.gr[i] > p4.gr[i]) d_near += (d_far - d_near) * ((O.gr[i] - p4.gr[i]) / (d_far * D.gr[i]));
		}
		else{
			if (pos > p4.gr[i]) d_far = d_near + (d_far - d_near) * ((p4.gr[i] - O.gr[i]) / (pos - O.gr[i]));
			if (O.gr[i] < p1.gr[i]) d_near += (d_far - d_near) * ((p1.gr[i] - O.gr[i]) / (d_far * D.gr[i]));
		}
		if (d_near > d_far) return 0;
	}
	
	Node* cur;
	cur = objs->tree->_root;
	stk[0].t = d_near; stk[0].pb = (d_near > 0.0f)?O + D * d_near:O;
	stk[1].t = d_far; stk[1].pb = O + D * d_far; stk[1].node = 0;
  int start_ = 0, end_ = 1;
  Node* d_ch;
	while (cur){
		while (!cur->IsLeaf()){
			real mid = cur->dpos;
			int axis = cur->ofs & 3;
      real stack_ex = stk[end_].pb.gr[axis];
			if(stk[start_].pb.gr[axis] <= mid){
				if(stack_ex <= mid){cur = cur->GetLeft();continue;}
				if(stack_ex == mid){cur = cur->GetRight();continue;}
				cur = cur->GetLeft(); d_ch = cur + 1;
			}
			else{
				if(stack_ex > mid){cur = cur->GetRight();continue;}
				d_ch = cur->GetLeft(); cur = d_ch + 1;
			}
			int tmp = end_++;
			if (end_ == start_) end_++;
			stk[end_].prev = tmp; stk[end_].t = (mid - O.gr[axis]) / D.gr[axis];
			stk[end_].node = d_ch; stk[end_].pb.gr[axis] = mid;
			stk[end_].pb.gr[_Mod[axis + 1]] = O.gr[_Mod[axis + 1]] + stk[end_].t * D.gr[_Mod[axis + 1]];
			stk[end_].pb.gr[_Mod[axis + 2]] = O.gr[_Mod[axis + 2]] + stk[end_].t * D.gr[_Mod[axis + 2]];
		}
		Obj_list* list = cur->GetList();
		real dist = stk[end_].t;
    int ans = 0;
		while (list){
			Abs_obj* pr = list->chara;
			int result;
			if (result = pr->Intersect( r_, dist )){
				ans = result; _dist = dist; N_obj = pr;
			}
			list = list->N_ext;
		}
		if (ans) return ans;
		start_ = end_;
		cur = stk[end_].node;
		end_ = stk[start_].prev;
	}
	return 0;
}

	int Search_(Ray& r_, real& _dist){
    Vec3 O = r_.from_, D = r_.dir_;
    int start_ = 0, end_ = 1;
    stk[start_].t = eps; stk[start_].pb = r_.from_;
    stk[end_].t = _dist; stk[end_].pb = O + D * _dist;
    stk[end_].node = NULL;
    Node *cur = objs->tree->_root, *d_ch;
    while (cur){
      while (!cur->IsLeaf()){
        real mid = cur->dpos;
        int axis = cur->ofs & 3;
        if (stk[start_].pb.gr[axis] <= mid){
          if (stk[end_].pb.gr[axis] <= mid){cur = cur->GetLeft();continue;}
          if (stk[end_].pb.gr[axis] == mid){cur = cur->GetRight();continue;}
          cur = cur->GetLeft(); d_ch = cur + 1;
        }
        else{
          if(stk[end_].pb.gr[axis] > mid){cur = cur->GetRight();continue;}
          d_ch = cur->GetLeft(); cur = d_ch + 1;
        }
        int tmp = end_++;
        if(end_ == start_) end_++;
        stk[end_].prev = tmp;
        stk[end_].t = (mid - O.gr[axis]) / D.gr[axis];
        stk[end_].node = d_ch;
        stk[end_].pb.gr[axis] = mid;
        stk[end_].pb.gr[_Mod[axis + 1]] = O.gr[_Mod[axis + 1]] + stk[end_].t * D.gr[_Mod[axis + 1]];
        stk[end_].pb.gr[_Mod[axis + 2]] = O.gr[_Mod[axis + 2]] + stk[end_].t * D.gr[_Mod[axis + 2]];
      }
      Obj_list* _top = cur->GetList();
      real dist = _dist;
      while(_top){
        if(_top->chara->Intersect(r_, dist)) return 1;
        _top = _top->N_ext;
      }
      start_ = end_;
      cur = stk[end_].node;
      end_ = stk[start_].prev;
    }
    return 0;
  }

	Abs_obj* Raytrace(Ray& _r, Color& show_color, int cur_dep, real R_id, real& _dist, real sp_num) {
    _dist = 10000.0f;//
    Abs_obj* near_obj = NULL;
    real scl_ = 1.0 / sp_num;
    Vec3 D;
    int F_N = Find_obj_s(_r, _dist, near_obj);
    if(!F_N) return NULL;
    Vec3 occ_p = _r.from_ + _r.dir_ * _dist;
    for(int pr = 0; pr < objs->num_lig; ++pr){
      Light* l = objs->Lig[pr];
      real shade = 0;
      D = l->GetPos() - occ_p;
      if (l->_id == 1){
        real len_ = D.Length();
        D *= 1.0 / len_;
        len_ *= (1 - 4 * eps);
        if (!Search_( Ray( occ_p + D * eps, D ), len_)) shade = 1.0;
      }
      else if (l->_id == 2){
        D.Normalize();
        for(int i = 0; i < sp_num; ++i) {
          Vec3 dir = l->gr[i&15] + R_mat.Rand() * l->d_x + R_mat.Rand() *  l->d_y - occ_p;
          real len_ = dir.Length();
          dir *= 1.0 / len_;
          len_ *= (1 - 4 * eps);
          if (!Search_( Ray( occ_p + dir * eps, dir ), len_)) shade += scl_;
        }
      }
      if(shade <= 0) continue;
      if(near_obj->mal_->diff_ > 0){
        real dot = DOT(D, near_obj->Norm_v(occ_p));
        if(dot > 0) show_color += dot*shade*near_obj->mal_->diff_*l->color_*near_obj->Get_color(occ_p);
      }

      if(near_obj->mal_->spec_ > 0){
        real dot = DOT(_r.dir_, (D - 2.0f * DOT(D, near_obj->Norm_v(occ_p)) * near_obj->Norm_v(occ_p)));
        if(dot > 0) show_color += near_obj->mal_->spec_ * shade / (50.0 / dot - 49.0) * l->color_;
      }
    }
    if(cur_dep > MAXDEP) return near_obj;
    real refle_ = near_obj->mal_->reflec_;
    if(refle_ > 0.0f){
      real drefl = near_obj->mal_->d_reflec;
      if((drefl > 0) && (cur_dep < 3)){
        Vec3 DR = _r.dir_ - 2.0f * DOT( _r.dir_, near_obj->Norm_v(occ_p) ) * near_obj->Norm_v(occ_p);
        refle_ *= scl_;
        for(int i = 0; i < SPNUM; ++i) {
          real vx, vy;
          while(1) {
            vx = (R_mat.Rand() - 0.5f) * drefl;
            vy = (R_mat.Rand() - 0.5f) * drefl;
            if((sqr(vx) + sqr(vy)) <= sqr(drefl)) break;
          }
          Vec3 H = Vec3(DR.z, DR.y, -DR.x);
          real dist;
          Color color_1(0, 0, 0);
          Vec3 R = DR + H * vx + DR.Cross(H) * vy * drefl;
          R.Normalize();
          Raytrace(Ray( occ_p + R * eps, R ), color_1, cur_dep + 1, R_id, dist, sp_num * 0.25f);
          show_color += refle_ * color_1 * near_obj->Get_color(occ_p);
        }
      }
      else {
        Vec3 R = _r.dir_ - 2.0f * DOT(_r.dir_, near_obj->Norm_v(occ_p)) * near_obj->Norm_v(occ_p);
        Color color_1(0, 0, 0);
        real dist;
        Raytrace(Ray(occ_p + R * eps, R ), color_1, cur_dep + 1, R_id, dist, sp_num * 0.5f);
        show_color += refle_ * color_1 * near_obj->Get_color(occ_p);
      }
    }
    if(near_obj->mal_->refrac_ > 0){
      real rindex = near_obj->mal_->ind_;
      real n = R_id / rindex;
      real cosI = -DOT((near_obj->Norm_v(occ_p) * (real)F_N), _r.dir_);
      real cosT2 = 1.0f - sqr(n) * (1.0f - sqr(cosI));
      if(cosT2 > 0.0f){
        Vec3 T = (n * _r.dir_) + (n * cosI - sqrtf(cosT2)) * near_obj->Norm_v(occ_p) * (real)F_N;
        Color color_2(0, 0, 0);
        real dist;
        Raytrace(Ray(occ_p + T * eps, T), color_2, cur_dep + 1, rindex, dist, sp_num * 0.5f);
        Color ab = near_obj->mal_->color_ * 0.15f * (-dist);
        show_color += color_2 * Color(expf(ab.r), expf(ab.g), expf(ab.b) );
      }
    }
    return near_obj;
  }


	void Start_rendering(){
		int pixel_ = 0;
    int red, green, blue;
		for(int y = 0; y < H; ++y){
      std::cout<<"rendering "<<100 * 1.0*y/H<<"%"<<std::endl;
			Vec3 pos_ = _P1 + 1.0f * y * m_DY;
			for(int x = 0; x < W; ++x){
        Vec3 dir = pos_ - from_;
				Color col_(0, 0, 0);
        dir.Normalize();
        Ray _r(from_, dir);
        real _dist(0);
        Abs_obj* prim = Raytrace(_r, col_, 1, 1.0f, _dist, SPNUM);
				red = std::min(255, (int)(col_.r * 255));
				green = std::min(255, (int)(col_.g * 255));
				blue = std::min(255, (int)(col_.b * 255));
        red_[pixel_] = red;
        green_[pixel_] = green;
        blue_[pixel_++] = blue;
				pos_ += m_DX;
			}
		}
}
};
#endif