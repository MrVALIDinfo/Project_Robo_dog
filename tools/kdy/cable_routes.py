"""Трассы кабелей серв: поиск пути по сетке 2 мм с зазором 2 мм от пластика (BFS), C++-модуль inside для «точка внутри детали».

Препятствия: рама и крышки, кронштейн плеча, бедро и его крышка, голень, корпуса серв, ось плеча (качалка и подшипник F625zz).
  бедро  — из торца серво вбок в щель у крышки бедра и по ноге к верху бедра (система бедра, стойка);
  корпус — от верха бедра и от вывода плечевого серво через щель у переборки I_shoulder к плате PCA9685 (base_link, стойка).
Колено проводится через ось колена с внешней стороны (там кабель не двигается при сгибании) — в app.js.
  g++ -O2 -o tools/kdy/inside tools/kdy/inside.cpp && python tools/kdy/cable_routes.py   -> 3d-viewer/parts/cable_routes.json
"""
import json, numpy as np, subprocess, collections, sys
from trimesh.transformations import euler_matrix, translation_matrix
from scipy import ndimage
import os
ROOT=os.path.abspath(os.path.join(os.path.dirname(__file__),'..','..'))


def plan(side):
  sy=1 if side=='l' else -1
  def inside(stl, M, P):
      inp = stl + '\n' + ' '.join(f'{x:.9g}' for x in np.asarray(M).ravel()) + f'\n{len(P)}\n' + '\n'.join(f'{a:.3f} {b:.3f} {c:.3f}' for a, b, c in P)
      return np.frombuffer(subprocess.run([ROOT + '/tools/kdy/inside'], input=inp, capture_output=True, text=True, check=True).stdout.strip().encode(), dtype=np.uint8) == ord('1')
  T=lambda xyz,rpy: translation_matrix(xyz)@euler_matrix(*rpy,'sxyz')
  S=json.load(open(f'{ROOT}/3d-viewer/parts/servo_seats.json'))
  VS=T([135,15 if sy>0 else 95,-10],[0,0,np.pi]); VL=T([130,-40 if sy>0 else 150,-25],[0,-0.139,np.pi]); VF=T([120,-40 if sy>0 else 150,100],[0,0,np.pi])
  f=lambda n: f'{ROOT}/3d-viewer/stl/kdy/{side}{n}.stl'
  def servo_M(n,Tv):
      s=S[side+n][0]; X,Z=np.array(s['x']),np.array(s['z']); Y=np.cross(Z,X); M=np.eye(4); M[:3,:3]=np.c_[X,Y,Z]; M[:3,3]=s['o']; return Tv@M
  def in_servo(P,M):
      l=(np.linalg.inv(M)@np.c_[P,np.ones(len(P))].T).T[:,:3]
      return (l[:,0]>-30.9)&(l[:,0]<10.6)&(np.abs(l[:,1])<10.6)&(l[:,2]>-38.5)&(l[:,2]<7)
  def grid(xs,ys,zs): G=np.stack(np.meshgrid(xs,ys,zs,indexing='ij'),-1); return G, G.reshape(-1,3)
  def bfs(G,free,start,goal):
      c=np.argwhere(free); s=tuple(c[np.argmin(np.linalg.norm(G[tuple(c.T)]-start,axis=1))]); prev={s:None}; q=collections.deque([s])
      while q:
          c=q.popleft()
          if goal(G[c]):
              out=[]
              while c is not None: out.append(G[c]); c=prev[c]
              return out[::-1]
          for d in [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)]:
              n=(c[0]+d[0],c[1]+d[1],c[2]+d[2])
              if all(0<=n[i]<G.shape[i] for i in range(3)) and free[n] and n not in prev: prev[n]=c; q.append(n)
  def pull(path, free, G):                         # спрямление по прямой видимости: самый дальний видимый узел пути, без «лесенки» сетки
      o=G[0,0,0]; h=G[1,0,0][0]-o[0] if G.shape[0]>1 else 2.0; h=abs(h)
      sgn=np.sign([G[-1,0,0][0]-o[0], G[0,-1,0][1]-o[1], G[0,0,-1][2]-o[2]]); sgn[sgn==0]=1
      def ok(a,b):
          n=int(np.ceil(np.linalg.norm(b-a)/0.5))+1
          for t in np.linspace(0,1,n):
              q=np.round((a+(b-a)*t-o)/(h*sgn)).astype(int)
              if (q<0).any() or (q>=free.shape).any() or not free[tuple(q)]: return False
          return True
      P=[np.array(q,float) for q in path]; out=[P[0]]; i=0
      while i<len(P)-1:
          j=len(P)-1
          while j>i+1 and not ok(P[i],P[j]): j-=1
          out.append(P[j]); i=j
      return [q.round(1).tolist() for q in out]
  def dp(path,tol=1.0):
      P=np.array(path)
      def rec(i,j):
          if j<=i+1: return [i,j]
          a,b=P[i],P[j]; ab=b-a; d=np.linalg.norm(np.cross(P[i+1:j]-a,ab),axis=1)/max(np.linalg.norm(ab),1e-9); k=int(d.argmax())
          return [i,j] if d[k]<=tol else rec(i,i+1+k)[:-1]+rec(i+1+k,j)
      return [P[i].round(0).astype(int).tolist() for i in rec(0,len(P)-1)]
  out={}
  # 1) внутри ноги, система бедра, колено -1.5
  Tf=translation_matrix([0,0,-107.5])@euler_matrix(0,-1.5,0)@VF
  Mh,Mk=servo_M('arm',VL),servo_M('foot',Tf)
  G,P=grid(np.arange(-46,46.1,2.),np.arange(-40,40.1,2.),np.arange(-150,36.1,2.))
  occ=(inside(f('arm'),VL,P)|inside(f('arm_cover'),VL,P)|inside(f('foot'),Tf,P)|in_servo(P,Mh)|in_servo(P,Mk)).reshape(G.shape[:3])
  goal=lambda p: p[2]>=16 and -30<=p[1]*sy<=-19 and abs(p[0])<=12
  def snap_bfs(free, start, goal, maxd=3.0):          # старт должен быть в свободной клетке не дальше maxd мм
      c=np.argwhere(free); d=np.linalg.norm(G[tuple(c.T)]-start,axis=1)
      return bfs(G,free,start,goal) if d.min()<=maxd else None
  for clr in (1,0):
      free=~ndimage.binary_dilation(occ,iterations=clr) if clr else ~occ
      p=snap_bfs(free,(Mh@np.array([-33,sy*11.5,-4,1]))[:3],goal)          # щель у торца серво, над фланцем
      if p: out['hip']=pull(p,free,G); out['hip_clear']=2*clr or 1; break
  Tfoot=translation_matrix([0,0,-107.5])@euler_matrix(0,-1.5,0); ax=(Tfoot@np.array([0,sy*30,-16,1]))[:3]
  for clr in (1,0):                                                         # колено: от вывода серво к оси колена (снаружи)
      free=~ndimage.binary_dilation(occ,iterations=clr) if clr else ~occ
      p=snap_bfs(free,(Mk@np.array([-31,0,-33,1]))[:3],lambda q: np.linalg.norm(q-ax)<3,maxd=4.5)
      if p: out['knee_down']=[(np.linalg.inv(Tfoot)@np.r_[q,1])[:3].round(1).tolist() for q in map(np.array,pull(p,free,G))]; out['knee_down_clear']=2*clr or 1; break
  top=np.array(out['hip'][-1]); tgoal=lambda p: np.linalg.norm(p-top)<1.5
  for clr in (1,0):                                                         # колено: от передней кромки бедра к верху бедра
      free=~ndimage.binary_dilation(occ,iterations=clr) if clr else ~occ
      p=snap_bfs(free,np.array([14,-sy*20,-80.]),tgoal)
      if p: out['knee_up']=pull(p,free,G); out['knee_clear']=2*clr or 1; break
  # 2) корпус: от верха бедра и от вывода плечевого серво к PCA, стойка
  for fx in (1,-1):
      Ts=translation_matrix([fx*93,sy*39,0]); Tleg=Ts@translation_matrix([0,sy*55,0])@euler_matrix(0,0.75,0)
      obs=[('mainbody',T([-42,-55,-10],[0,0,0])),('frontpart',T([40,55,-10],[0,0,np.pi])),('backpart_alatoo',T([40,55,-10],[0,0,np.pi]))]
      G,P=grid(np.arange(0,136,2.)*fx,np.arange(-4,106,2.)*sy,np.arange(-44,40.1,2.))
      o=np.zeros(len(P),bool)
      for n,M in obs: o|=inside(f'{ROOT}/3d-viewer/stl/kdy/{n}.stl',M,P)
      o|=inside(f('shoulder'),Ts@VS,P)|inside(f('arm'),Tleg@VL,P)|inside(f('arm_cover'),Tleg@VL,P)|in_servo(P,servo_M('shoulder',Ts@VS))|in_servo(P,servo_M('arm',Tleg@VL))
      o|=(np.abs(P[:,0])>=55)&(np.abs(P[:,0])<=125)&(np.hypot(P[:,1]-sy*39,P[:,2]-8)<9)
      free=~ndimage.binary_dilation(o.reshape(G.shape[:3]),iterations=1)
      g2=lambda p: abs(p[0])<=50 and abs(p[1])<=30 and 0<=p[2]<=16
      k='front' if fx>0 else 'rear'
      for nm,st in [('leg',(Tleg@np.array([*out['hip'][-1],1]))[:3]),('shoulder',(servo_M('shoulder',Ts@VS)@np.array([-31,0,-33,1]))[:3])]:
          p=bfs(G,free,st,g2); out[f'{k}_{nm}']=pull(p,free,G) if p else None
  return out


if __name__ == '__main__':
    json.dump({s: plan(s) for s in ('l', 'r')}, open(os.path.join(ROOT, '3d-viewer', 'parts', 'cable_routes.json'), 'w'))
    print('готово: 3d-viewer/parts/cable_routes.json')
