/*

				Terrain Marching - 2017
				Shader de rendu des montagnes enneigées en terrain marching
				La couleur d'ambiance est: couleur_zénith * intensite_ambiante


*/

in {
	tex2D texture_terrain [wrap-u: repeat, wrap-v: repeat];
	tex2D texture_terrain2 [wrap-u: repeat, wrap-v: repeat];
	tex2D texture_terrain3 [wrap-u: repeat, wrap-v: repeat];
	tex2D texture_ciel [wrap-u: clamp, wrap-v: clamp];
	float ratio_ecran;
	float distanceFocale;
	vec3 obs_pos;
	mat3 obs_mat_normale;
	vec2 facteur_echelle_terrain1;
	vec2 facteur_echelle_terrain2; 
	vec2 facteur_echelle_terrain3; 
	float amplitude_terrain1;
	float amplitude_terrain2;
	float amplitude_terrain3;
	vec3 offset_terrain;
	vec3 couleur_horizon;
	vec3 couleur_zenith;
	vec3 couleur_neige;
	vec3 couleur_eau;
	vec3 couleur_cristaux;
	vec3 couleur_mineral1;
	vec3 couleur_mineral2;
	vec3 speculaire_mineral;
	float brillance_mineral;
	vec3 speculaire_neige;
	float brillance_neige;
	vec2 mineral_fading;
	float intensite_ambiante;
	float altitude_eau;
	float reflexion_eau;
	float temps;
	
	vec3  l1_direction;
    vec3  l1_couleur;
    vec3  l2_direction;
    vec3  l2_couleur;
	vec2 zFrustum;
	
}

variant {
	
	//====================================================================================================
	
	vertex {
		out {
			vec2 v_uv;
			vec3 rayObs_dir; 
		}

		source %{
			v_uv = vUV0;
			rayObs_dir=vec3(vPosition.x,vPosition.y,0.)*vec3(ratio_ecran,1.,0.)+vec3(0.,0.,distanceFocale);
			%out.position% = vec4(vPosition, 1.0);
		%}
	}

	//====================================================================================================
	
	pixel {
			
		global %{
			
			#define EPSILON_NORMALE 0.2f
			#define FACTEUR_PSILON_NORMALE 1.005f
			#define M_PI 3.141592653
			#define FACTEUR_GRIS_R 0.11
			#define FACTEUR_GRIS_V 0.59
			#define FACTEUR_GRIS_B 0.3


			float calcule_zDepth(float near,float far)
			{
					float a,b,z;
					z=far*near;
					a=zFrustum.y/(zFrustum.y-zFrustum.x);
					b=zFrustum.y*zFrustum.x/(zFrustum.x-zFrustum.y);
					return ((a+b/z)+1.)/2.;
			}
			
			
			float renvoie_altitude(vec2 p)
			{
				p -= offset_terrain.xz;
				float a=texture2D(texture_terrain,p*facteur_echelle_terrain1).r;
				float b=texture2D(texture_terrain2,p*facteur_echelle_terrain2).r;//12,753
				float c=texture2D(texture_terrain3,p*facteur_echelle_terrain3).r;

				return (pow(a,5.)*amplitude_terrain1+pow(b,4.)*amplitude_terrain2+c*amplitude_terrain3)+offset_terrain.y;
			}
			
			vec3 calcul_normale(vec3 pos, float d)
			{
				float f=clamp(EPSILON_NORMALE*pow(FACTEUR_PSILON_NORMALE,d),EPSILON_NORMALE,10.);
				vec2 xd=vec2(f,0);
				vec2 zd=vec2(0,f);
				return normalize(vec3(
										renvoie_altitude(pos.xz-xd) - renvoie_altitude(pos.xz+xd),
										2.*f,
										renvoie_altitude(pos.xz-zd) - renvoie_altitude(pos.xz+zd)
									  ));
			}
			
			
			// ===================== Surface de l'eau:
			
			float renvoie_texel_eau(vec2 pos)
			{
				float a=texture2D(texture_terrain3,(pos+vec2(10.1,2.4)*temps)*vec2(1./1000.,1./1000.)).r;
				return a*1.5;
			}
			
			vec3 renvoie_normale_eau(vec3 pos)
			{
				float f=5.;
				return normalize(vec3(
										renvoie_texel_eau(vec2(pos.x-f,pos.z)) - renvoie_texel_eau(vec2(pos.x+f,pos.z)),
										2.*f,
										renvoie_texel_eau(vec2(pos.x,pos.z-f)) - renvoie_texel_eau(vec2(pos.x,pos.z+f))
									  ));
			}
			
			//================ Calcul éclairage:
			
			vec2 calcul_eclairage_lineaire(vec3 obs_direction, vec3 light_direction, vec3 normale, float brillance)
			{
				float diffuse  = max(dot(normale,light_direction),0.);
				vec3 reflet_source	=   normalize(-obs_direction+light_direction);
				float speculaire = pow(max(dot(reflet_source,normale),0.),brillance);
				return vec2(diffuse,speculaire);
			}
			
			vec3 eclairage_materiau(vec3 obs_direction, vec3 light_direction, vec3 normale, vec3 c_light, vec3 c_diffuse, vec3 c_speculaire, float brillance)
			{
				vec2 eclairage = calcul_eclairage_lineaire(obs_direction,normale,light_direction,brillance);
				return vec3( c_diffuse * c_light * eclairage.x + c_speculaire * c_light * eclairage.y );
			}
			
			
			vec3 eclairage_neige(vec3 position, vec3 obs_direction, vec3 light_direction[2], vec3 normale, vec3 c_light[2], vec3 c_ambiant)
			{
				vec3 luminosite = couleur_cristaux * texture2D(texture_terrain3,position.xz*facteur_echelle_terrain3).g;
				vec3 c=vec3(0,0,0);
				for (int i=0;i<2;i++) c+=eclairage_materiau(obs_direction,-light_direction[i],normale,c_light[i],couleur_neige, speculaire_neige, brillance_neige);
				
				return clamp(c_ambiant + luminosite + c,vec3(0,0,0),vec3(1,1,1));
			}
			
			vec3 eclairage_mineral(vec3 position, vec3 obs_direction, vec3 light_direction[2], vec3 normale, vec3 c_light[2], vec3 c_ambiant)
			{
				vec3 couleur_mineral = mix(couleur_mineral2,couleur_mineral1,clamp((position.y-mineral_fading.x)/(mineral_fading.y-mineral_fading.x),0.,1.));
				vec3 c=vec3(0,0,0);
				for (int i=0;i<2;i++) c+=eclairage_materiau(obs_direction,-light_direction[i],normale,c_light[i],couleur_mineral, speculaire_neige, brillance_mineral);
				return clamp(c_ambiant + c,vec3(0,0,0),vec3(1,1,1));
			}
			
			vec3 eclairage(vec3 position, vec3 obs_direction, vec3 light_direction[2], vec3 normale, vec3 c_light[2])
			{
				vec3 couleur;
				vec3 c_ambiant=couleur_zenith*intensite_ambiante;
				if (normale.y>0.85)
				{
					couleur = eclairage_neige(position,obs_direction,light_direction,normale,c_light,c_ambiant);
				}
				else
				{
					couleur = eclairage_mineral(position,obs_direction,light_direction,normale,c_light,c_ambiant);
				}
				return couleur;
			}
			
			//===================================== renvoie un élément d'un texture en coordonnées sphériques:
			// La direction doit être normalisée
			vec3 get_sky_texel(vec3 dir)
			{
				float phi,theta,v,r;
				vec2 uv;
				if (dir.y<0) return vec3(0.,0.,0.);
				v=acos(dir.x/sqrt(pow(dir.x,2)+pow(dir.z,2)));
				phi=asin(dir.y);
				if (dir.z>=0.)
				{
					theta=v;
				}
				else
				{
					theta=2.*M_PI-v;
				}
				r=0.5-phi/M_PI;
				//r=0.5-abs(cos(phi))/2.;
				uv=vec2(r*cos(theta),r*sin(theta))+vec2(0.5,0.5);
				return texture2D(texture_ciel,uv).rgb;
			}
			
			vec3 renvoie_couleur_atmosphere(vec3 ray_dir, vec3 c_horizon, vec3 c_zenith)
			{
				return mix(c_zenith,c_horizon,pow(min(1.,1.-abs(ray_dir.y)),2.));
			}
			
			float renvoie_intensite_soleil(vec3 ray_dir, vec3 sun_dir)
			{
				float prod_scal = max(dot(sun_dir,-ray_dir),0);
				return min(pow(prod_scal,8000) + 0.2 * pow(prod_scal,500) + 0.5 *pow(prod_scal,10),1);
			}
			
			
			vec3 get_sky_color(vec3 dir,vec3 sun_dir,vec3 c_sun, vec3 c_atmosphere)
			{
				vec3 sky_tex=get_sky_texel(dir);
				float sun_lum = renvoie_intensite_soleil(dir,sun_dir);
				float sky_lum=c_atmosphere.r*FACTEUR_GRIS_R+c_atmosphere.g*FACTEUR_GRIS_V+c_atmosphere.b*FACTEUR_GRIS_B;
				//float luminosite_texture=texel_ciel.r*FACTEUR_GRIS_R+texel_ciel.g*FACTEUR_GRIS_V+texel_ciel.b*FACTEUR_GRIS_B;
				return mix(c_atmosphere+sky_tex*(1-sky_lum),c_sun,sun_lum);
			}
			
			vec3 get_ground_color(vec3 pos,vec3 dir, float d,vec3 l_dir[2],vec3 c_light[2],vec3 c_atmosphere)
			{
				vec3 color=eclairage(pos, dir, l_dir, calcul_normale(pos, d), c_light);
				float f=clamp((d-zFrustum.x)/(zFrustum.y*0.999-zFrustum.x),0.,1.);
				return mix(color,c_atmosphere,f);
			}
			
			float intersect_up(vec3 pos, vec3 dir,float alt_max)
			{
				float dist,alt;
				float s=0.;
				vec3 p;
				for (dist=zFrustum.x;dist<zFrustum.y;dist+=s)
				{
					p=pos+dir*dist;
					if (p.y>=alt_max) return zFrustum.y;
					alt = p.y - renvoie_altitude(p.xz);
					if (alt<0.002*dist) return dist;
					s=0.3*alt;
				}
				return dist;
			}
			
			float intersect_down(vec3 pos, vec3 dir,float dist_max)
			{
				float dist,alt;
				float s=0.;
				vec3 p;
				for (dist=zFrustum.x;dist<zFrustum.y;dist+=s)
				{
					if (dist>dist_max) return dist_max;
					p=pos+dir*dist;
					alt = p.y - renvoie_altitude(p.xz);
					if (alt<0.002*dist) return dist;
					s=0.3*alt;
				}
				return dist;
			}
			
			
			vec3 get_color(vec3 pos, vec3 dir, float d, vec3 l_dir[2], vec3 c_light[2], vec3 c_atmosphere,float alt_max)
			{
				if (d>=zFrustum.y*0.999) return get_sky_color(dir,l_dir[0],c_light[0],c_atmosphere);
				else 
				{
					float lighting=1.;
					vec3 c_l[2];
					c_l[1]=c_light[1];
					float dist_shadow=intersect_up(pos,-l_dir[0],alt_max);
					if (dist_shadow<zFrustum.y*0.999) lighting=0.;
					c_l[0]=c_light[0]*lighting;
					return get_ground_color(pos,dir,d,l_dir,c_l,c_atmosphere);
				}
			}
			
		%}
		
	//====================================================================================================
		
		
		source %{
			vec3 couleur;
			float distance_eau;
			float altitude_max=abs(amplitude_terrain1)+abs(amplitude_terrain2)+abs(amplitude_terrain3);
			vec3 obs_pos_march = vec3(obs_pos);
			//Rayon:
			vec3 obsDir=normalize(rayObs_dir);

			//Calcul la position et le vecteur directeur du rayon dans l'espace absolu (pour le moment l'observateur est dans le même repère que l'espace)
			vec3 ray_dir=normalize(obs_mat_normale*normalize(rayObs_dir));

			//Couleur ciel:
			vec3 couleur_atmosphere = renvoie_couleur_atmosphere(ray_dir,couleur_horizon,couleur_zenith);
			
			//Marche:
			vec3 l_dir[2];
			l_dir[0]=l1_direction;
			l_dir[1]=l2_direction;
			vec3 c_lights[2];
			c_lights[0]=l1_couleur;
			c_lights[1]=l2_couleur;
		
			float dist;
			if (ray_dir.y>=0) 
			{
				dist = intersect_up(obs_pos,ray_dir,altitude_max);
				couleur = get_color(obs_pos+ray_dir*dist, ray_dir, dist, l_dir, c_lights, couleur_atmosphere,altitude_max);
			}
			else 
			{
				
				float distance_eau = (obs_pos.y - altitude_eau) / dot(vec3(0,-1,0),ray_dir);
				dist = intersect_down(obs_pos,ray_dir,distance_eau);
				
				if (dist>distance_eau*0.999 && dist <distance_eau*1.001)
				{
					vec3 pos_r=obs_pos+ray_dir*dist;
					vec3 dir_r=reflect(ray_dir,renvoie_normale_eau(pos_r));
					float dist_r=intersect_up(pos_r,dir_r,altitude_max);
					
					couleur = get_color(pos_r+dir_r*dist_r, dir_r, dist_r + dist,l_dir, c_lights, couleur_atmosphere,altitude_max);
					
					float f=clamp((dist-zFrustum.x)/(zFrustum.y*0.999-zFrustum.x),0.,1.);
					couleur=mix(mix(couleur_eau,couleur_atmosphere,f),couleur,reflexion_eau);
				}
				else
				{
					couleur = get_color(obs_pos+ray_dir*dist, ray_dir, dist, l_dir, c_lights, couleur_atmosphere,altitude_max);
				}
				
				
			}
			
			%out.color% =vec4(couleur,1.); 
			%out.depth%=calcule_zDepth(obsDir.z,min(dist,zFrustum.y*0.999));
			
		%}
	}
}
