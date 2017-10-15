#include <iostream>
#include <cstdio>
#include <cstring>
#include "render.h"
#include "constant.h"

void Space::Mk_home() {
  Attr* mat;
  mat = new Attr();
  mat->SetPrams(0.2, 0.0, Color(0.9, 0.9, 0.9), 1.0, 2.0); 
  Mk_plane(Vec3(-10.0, -5.0, -10.0), Vec3(10.0, -5.0, -10.0),Vec3(10.0, -5.0, 10.0), Vec3(-10.0, -5.0f, 10.0), mat);
  mat = new Attr();
  mat->SetPrams(0.2, 0.0, Color(0.6, 0.6, 0.6), 2.0, 0.0); 
  Mk_plane(Vec3(10.0, -5.0, -10.0), Vec3(10.0, -5.0, 10),Vec3(10.0, 10.0, 10), Vec3( 10.0, 10.0, -10.0), mat);
  mat = new Attr();
  mat->SetPrams(0.0, 0.0, Color(0.1, 0.6, 0.2), 2.0, 3.0);
  Mk_plane(Vec3(-10.0, -5.0, 10.0), Vec3(10.0, -5.0, 10),Vec3(10.0, 10.0, 10), Vec3(-10.0, 10.0, 10.0), mat);
  mat = new Attr();
  mat->SetPrams(0.0, 0.0, Color(0.9, 0.9, 0.9), 1.6, 0.0);
  Mk_plane(Vec3(10.0, 10.0, -10.0), Vec3(-10.0, 10.0, -10.0),Vec3(-10.0, 10.0, 10.0), Vec3(10.0, 10.0, 10.0), mat);
}

void Space::Mk_light() {
  //Lig[num_lig++] = new Light(1, Vec3(0, 3, 5), Color(0.4, 0.4, 0.4) );
  //Lig[num_lig++] = new Light(1, Vec3(-2, 5, 1), Color(0.6, 0.6, 0.8 ) );
  //Lig[num_lig++] = new Light(1, Vec3(-3, 5, 1 ), Color( 0.6, 0.6, 0.8 ) );

  Lig[num_lig++] = new Light( 2, Vec3( -1, 6, 4 ), Vec3( 1, 6, 4 ), Vec3( -1, 6, 6 ), Color( 0.7, 0.7, 0.7 ) );
  Lig[num_lig++] = new Light( 2, Vec3( -1, 6, -1 ), Vec3( 1, 6, -1 ), Vec3( -1, 6, 1 ), Color( 0.7, 0.7, 0.7 ) );
  
}
void Space::InitSpace(std::vector<real> &w1, std::vector<real> &w2, std::vector<real> &w3, std::vector<int> &j1, std::vector<int> &j2, std::vector<int> &j3){
    Attr* mat;
    num_lig = charas = 0;
    chara = new Abs_obj*[200000];
    Lig = new Light*[100];
    
    Mk_home();
    
    chara[charas] = new Abs_obj(1, Vec3(2.0, 0.0, 0.0), 2);
    chara[charas]->mal_->SetPrams(0.1, 0.0, Color(0.6, 0.6, 0.6), 1.0, 1.0);
    chara[charas++]->mal_->SetTexture(new Texture("textures/1.tga"));
    chara[charas] = new Abs_obj(1, Vec3(-2.0, -1.0, 4.0), 2);
    chara[charas]->mal_->SetPrams(0.2, 1, Color(0.1, 0.1, 0.1), 0.2, 1.8);
    chara[charas++]->mal_->SetRefrInd(1.1);
    
    Mk_light();
    
    mat = new Attr();
    mat->SetPrams(0.0, 0.0, Color(0.2, 1.0, 0.2), 0.6, 0.9);
    load_from_obj(mat, 1 , w1, w2, w3, j1, j2, j3);
    Frw = aabb(Vec3(-20, -10, -10), Vec3(40, 20, 40));
    tree = new KdTree(this);
}
void Space::Mk_plane( Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3, Attr* _mat ){
  Vex *v0 = new Vex(p0, 0, 0), *v1 = new Vex(p1, 0, 0), *v2 = new Vex(p2, 0, 0), *v3 = new Vex(p3, 0, 0);
  chara[charas] = new Abs_obj(2, v0, v1, v3);
  chara[charas++]->mal_ = _mat;
  chara[charas] = new Abs_obj(2, v1, v2, v3);
  chara[charas++]->mal_ = _mat;
}
	
void Space::load_from_obj(Attr* a_Attr, real a_Size, std::vector<real> &w1, std::vector<real> &w2, std::vector<real> &w3, std::vector<int> &j1, std::vector<int> &j2, std::vector<int> &j3) {
    char* buf = new char[256];
    std::vector<Vex*> v;
    std::vector<Vec3*> n;
    real base = 50.0;
    num_V = 0;
    int sum = 0;
    Vec3 min(1000000, 100000, 100000), max, size, center;

    for(int i = 0; i < w1.size(); ++i) {
        double x, y, z;
        x = w1[i];
        y = w2[i];
        z = w3[i];
        min.x = std::min(min.x, real(x));
        min.y = std::min(min.y, real(y));
        min.z = std::min(min.z, real(z));
        max.x = std::max(max.x, real(x));
        max.y = std::max(max.y, real(y));
        max.z = std::max(max.z, real(z));
        
        ++num_V;
        real tx=x, ty=y, tz=z;
        x = tx * 2;
        y = ty * 2;
        z = tz * 2 + 3;

        v.push_back(new Vex(x, y, z));
        v[num_V-1]->SetUV( v[v.size()-1]->GetPos().gr[0] / a_Size, v[v.size()-1]->GetPos().gr[1] / a_Size );
        n.push_back(new Vec3());

      } 
      for(int i = 0; i < j1.size(); ++i) {
        
        int x, y, z;
        x = j1[i];
        y = j2[i];
        z = j3[i];

        chara[charas] = new Abs_obj( 2, v[x-1], v[y-1], v[z-1] );
        chara[charas++]->mal_ = a_Attr;
        *(n[x-1]) = *(n[x-1]) + chara[charas - 1]->N_;
        *(n[y-1]) = *(n[y-1]) + chara[charas - 1]->N_;
        *(n[z-1]) = *(n[z-1]) + chara[charas - 1]->N_;

      }

    size = max - min;
    center = (min + max) * 0.5f;
    std::cout<<size.x<<' '<<size.y<<' '<<size.z<<std::endl;
    std::cout<<center.x<<' '<<center.y<<' '<<center.z<<std::endl;
    //std::cin>>sum;
    for (int i = 0; i < num_V; i++ )
    {
      Vec3 tmp = *(n[i]);
      tmp.Normalize();
      v[i]->N_ = tmp;
    }
}
void KdTree::Build_tree(Space* _objs){
	for(int pr = 0; pr < _objs->charas; ++pr) _root->Add(_objs->chara[pr]);
	int size_ = _objs->charas;
	_spool = new D_list[size_ * 2 + 8];
	int i = 0;
  while(i < (size_ * 2 + 6)) {
    _spool[i].next = &_spool[i + 1];
    ++i;
  }
	_spool[i].next = _slist = NULL;
	Down_(_root, _objs->Frw, 0, size_);
}

void KdTree::Insert_(real mid_){
	D_list* pr = _spool;
  pr->pos_ = mid_;
	_spool = _spool->next;
	pr->next = NULL;
  pr->tot_l = pr->tot_r = 0;
	if (!_slist) _slist = pr;
  else {
		if(mid_ <= _slist->pos_){
      if(mid_ == _slist->pos_){pr->next = _spool; _spool = pr;return;}
      pr->next = _slist; _slist = pr;
		}
		else {
			D_list* list = _slist;
			for (;(list->next) && (mid_ >= list->next->pos_);list = list->next)
				if (mid_ == list->next->pos_){
					pr->next = _spool;
					_spool = pr;
					return;
				}
			pr->next = list->next;
			list->next = pr;
		}
	}
}
Creater* KdTree::s_Creater = 0;

void Node::Add(Abs_obj* _O) {
	Obj_list* lnode = KdTree::s_Creater->NewObj_list();
	lnode->chara = _O;
	lnode->Next(GetList());
	SetList(lnode);
}