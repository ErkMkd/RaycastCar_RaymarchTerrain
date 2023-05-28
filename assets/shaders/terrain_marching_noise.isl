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
	float facteur_precision_distance;
	vec3 couleur_horizon;
	vec3 couleur_zenith;
	vec3 couleur_neige;
	vec3 couleur_eau;
	vec3 couleur_cristaux;
	float intensite_ambiante;
	float altitude_eau;
	float reflexion_eau;
	
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
			
			#define EPSILON_NORMALE 0.5f
			#define FACTEUR_PSILON_NORMALE 1.001f
			#define M_PI 3.141592653
			#define FACTEUR_GRIS_R 0.11
			#define FACTEUR_GRIS_V 0.59
			#define FACTEUR_GRIS_B 0.3
			struct Ray
			{
				vec3 pos;
				vec3 dir;
			};
			
			Ray rayObs;
			
			vec3 obsDir,normale_terrain,couleur,couleur_ciel;
			float dist;
			float distance_eau;
			float facteur_brouillard;
			vec3 l1d,l2d;
			float brouillard_min=10.;
			vec3 couleur_ambiante=couleur_zenith*intensite_ambiante;
			float altitude_max=abs(amplitude_terrain1)+abs(amplitude_terrain2)+abs(amplitude_terrain3);
			vec3 obs_pos_march = vec3(obs_pos);
			
			float calcule_zDepth(float d)
			{
					float a,b,z;
					z=d*obsDir.z;
					a=zFrustum.y/(zFrustum.y-zFrustum.x);
					b=zFrustum.y*zFrustum.x/(zFrustum.x-zFrustum.y);
					return ((a+b/z)+1.)/2.;
			}
			
			vec3 hash(vec3 p)
			{
					p = vec3( dot(p,vec3(123.21,311.7, 74.7)),
							  dot(p,vec3(61.46,183.3,246.1)),
							  dot(p,vec3(47.654,271.9,124.6)));

					return -1.0 + 2.0*fract(sin(p)*134.53123);
			}
			
			
			float noise( vec3 x )
			{
				// grid
				vec3 p = floor(x);
				vec3 w = fract(x);
				
				// quintic interpolant
				vec3 u = w*w*w*(w*(w*6.0-15.0)+10.0);

				
				// gradients
				vec3 ga = hash( p+vec3(0.0,0.0,0.0) );
				vec3 gb = hash( p+vec3(1.0,0.0,0.0) );
				vec3 gc = hash( p+vec3(0.0,1.0,0.0) );
				vec3 gd = hash( p+vec3(1.0,1.0,0.0) );
				vec3 ge = hash( p+vec3(0.0,0.0,1.0) );
				vec3 gf = hash( p+vec3(1.0,0.0,1.0) );
				vec3 gg = hash( p+vec3(0.0,1.0,1.0) );
				vec3 gh = hash( p+vec3(1.0,1.0,1.0) );
				
				// projections
				float va = dot( ga, w-vec3(0.0,0.0,0.0) );
				float vb = dot( gb, w-vec3(1.0,0.0,0.0) );
				float vc = dot( gc, w-vec3(0.0,1.0,0.0) );
				float vd = dot( gd, w-vec3(1.0,1.0,0.0) );
				float ve = dot( ge, w-vec3(0.0,0.0,1.0) );
				float vf = dot( gf, w-vec3(1.0,0.0,1.0) );
				float vg = dot( gg, w-vec3(0.0,1.0,1.0) );
				float vh = dot( gh, w-vec3(1.0,1.0,1.0) );
				
				// interpolation
				return va + 
					   u.x*(vb-va) + 
					   u.y*(vc-va) + 
					   u.z*(ve-va) + 
					   u.x*u.y*(va-vb-vc+vd) + 
					   u.y*u.z*(va-vc-ve+vg) + 
					   u.z*u.x*(va-vb-ve+vf) + 
					   u.x*u.y*u.z*(-va+vb+vc-vd+ve-vf-vg+vh);
			}
			
			
			float renvoie_altitude_details(vec2 p)
			{
				vec3 pa = vec3(p.x,0,p.y);
				float a=noise(pa*facteur_echelle_terrain1.x);

				float b=noise(pa*facteur_echelle_terrain2.x);
				float c=noise(pa*facteur_echelle_terrain3.x);

				return (pow(a,5.)*amplitude_terrain1+pow(b,4.)*amplitude_terrain2+c*amplitude_terrain3);
				//return (a+b+c)*amplitude_terrain1;
			}
			
			void calcul_normale(float d)
			{
				float f=EPSILON_NORMALE;//clamp(EPSILON_NORMALE*pow(FACTEUR_PSILON_NORMALE,d),EPSILON_NORMALE,1.);
				normale_terrain=normalize(vec3(
										renvoie_altitude_details(vec2(rayObs.pos.x-f,rayObs.pos.z)) - renvoie_altitude_details(vec2(rayObs.pos.x+f,rayObs.pos.z)),
										2.*f,
										renvoie_altitude_details(vec2(rayObs.pos.x,rayObs.pos.z-f)) - renvoie_altitude_details(vec2(rayObs.pos.x,rayObs.pos.z+f))
									  ));
			}
			
			//================ Calcul éclairage:
			
			vec3 calcul_eclairage(vec3 normale)
			{
				vec3 reflet_source;

				float angle_source1_normale  = max(dot(normale,l1d),0.);
				float angle_source2_normale  = max(dot(normale,l2d),0.);


				vec3 materiau_luminosite;
				vec3 materiau_diffusion;
				vec3 materiau_specularite;
				float materiau_brillance;

				if(normale.y>0.85)
				{
					float c=texture2D(texture_terrain3,rayObs.pos.xz*facteur_echelle_terrain3).g;
					/*
					if(c<0.15)
					{
					   materiau_luminosite=max(1.-dist/500.,0.)*couleur_neige;
					}
					else
					{
						materiau_luminosite=vec3(0.,0.,0.);
					}
					*/
					materiau_luminosite=vec3(c,c,c)*couleur_cristaux;
					materiau_diffusion=couleur_neige;
					materiau_specularite=vec3(.5,.5,.5);
					materiau_brillance=90.;
				}
				else
				{
					vec3 couleur_mineral1=vec3(0.3,0.3,0.3);
					vec3 couleur_mineral2=vec3(122./255.,105./255.,95./255.);
					materiau_luminosite=vec3(0.,0.,0.);
					materiau_diffusion=mix(couleur_mineral2,couleur_mineral1,clamp((rayObs.pos.y-50.)/(200.-50.),0.,1.));
					materiau_specularite=vec3(.1,.1,.1);
					materiau_brillance=30.;
				}


				vec3 eclairage    = 	materiau_luminosite + couleur_ambiante; //La couleur utilisée pour l'ambiance

				eclairage    += 	materiau_diffusion * (l1_couleur *  angle_source1_normale
													 + l2_couleur *  angle_source2_normale
													);



				if (angle_source1_normale>0.)
				{
					reflet_source	=   normalize(-rayObs.dir+l1d);
					eclairage    +=   l1_couleur * materiau_specularite * pow(max(dot(reflet_source,normale),0.),materiau_brillance);
				}

				if (angle_source2_normale>0.)
				{
					reflet_source	=   normalize(-rayObs.dir+l2d);
					eclairage    +=   l2_couleur * materiau_specularite * pow(max(dot(reflet_source,normale),0.),materiau_brillance);
				}


				return eclairage;
			}
			
			
			//===================================== Calcul distance précise:
			
			float renvoie_dist_precise(float d0,float d1)
			{
				float d_mid,alt_mid;
				vec3 ray;

				for(int i=0;i<2;i++)
				{
					d_mid=d0+(d1-d0)/2;
					ray=obs_pos_march+rayObs.dir*d_mid;
					alt_mid=renvoie_altitude_details(ray.xz);
					if(alt_mid<ray.y) d0=d_mid;
					else if(alt_mid>ray.y) d1=d_mid;
					else break;
				}
				return d_mid;
				
				

				/*
				vec3 ray0=obs_pos+rayObs.dir*d0;
				vec3 ray1=obs_pos+rayObs.dir*d1;
				float alt0=renvoie_altitude_details(ray0.xz);
				float alt1=renvoie_altitude_details(ray1.xz);
				float d_alt0=ray0.y-alt0;
				float d_alt1=alt1-ray1.y;
				return d0+(d1-d0)*(d_alt0/(d_alt0+d_alt1));
				*/
				
				

				/*
				float ray0=obs_pos.y+rayObs.dir.y*d0;
				float ray1=obs_pos.y+rayObs.dir.y*d1;
				float d_alt0=ray0-a0;
				float d_alt1=a1-ray1;
				return d0+(d1-d0)*(d_alt0/(d_alt0+d_alt1));*/
			}
			
			//===================================== renvoie un élément d'un texture en coordonnées sphériques:
			// La direction doit être normalisée
			vec3 renvoie_texel_ciel(vec3 dir)
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
			
		%}
		
	//====================================================================================================
		
		
		source %{
			
			//Rayon:

			obsDir=normalize(rayObs_dir);
			dist=zFrustum.y;

			//Calcul la position et le vecteur directeur du rayon dans l'espace absolu (pour le moment l'observateur est dans le même repère que l'espace)
				rayObs.dir=normalize(obs_mat_normale*obsDir);
				l1d=-l1_direction;
				l2d=-l2_direction;

			//Couleur ciel:
			couleur_ciel=mix(couleur_zenith,couleur_horizon,pow(min(1.,1.-rayObs.dir.y),2.));
			float angle_soleil=pow(max(dot(l1d,rayObs.dir),0.),512.);
			couleur_ciel=mix(couleur_ciel,l1_couleur,angle_soleil);

			//Marche:
				if (rayObs.dir.y<0)
				{
					distance_eau = (obs_pos_march.y - altitude_eau) / dot(vec3(0,-1,0),rayObs.dir);
				}
				float drapeau_reflet_eau=0;
				float alt,alt_prec,dist_prec;
				rayObs.pos=obs_pos_march+rayObs.dir*zFrustum.x;
				float pas=.1;
				if(rayObs.dir.y<0.99)
				{
					
					for(dist=zFrustum.x;dist<zFrustum.y;dist+=pas)
					{

						rayObs.pos += rayObs.dir * pas;
						if(rayObs.dir.y>0. && rayObs.pos.y>altitude_max)
						{
							dist=zFrustum.y;
							break;
						}
						

						alt=renvoie_altitude_details(rayObs.pos.xz);
	
						if (rayObs.pos.y<altitude_eau && drapeau_reflet_eau==0)
						{
								
								rayObs.pos = obs_pos_march + rayObs.dir * distance_eau;
								dist = distance_eau;
								alt = renvoie_altitude_details(rayObs.pos.xz);
								if (rayObs.pos.y>alt)
								{
									rayObs.dir=reflect(rayObs.dir,vec3(0.,1.,0.));
									obs_pos_march.y = 2*altitude_eau - obs_pos_march.y;
									drapeau_reflet_eau=1.;
								}
						}
	
						if (alt>rayObs.pos.y)
						{
							//if (drapeau_reflet_eau==0)
							//{
								dist=renvoie_dist_precise(dist_prec,dist);
								rayObs.pos=obs_pos_march+rayObs.dir*dist;
							//}

			
							calcul_normale(dist);
							

							couleur=calcul_eclairage(normale_terrain);

							break;
						}
						
						
						
						pas*=facteur_precision_distance;
						dist_prec=dist;
						alt_prec=alt;
					}
				} 
				
				if (dist>zFrustum.y*0.999)
				{
					vec3 texel_ciel=renvoie_texel_ciel(rayObs.dir);
					float luminosite_ciel=couleur_ciel.r*FACTEUR_GRIS_R+couleur_ciel.g*FACTEUR_GRIS_V+couleur_ciel.b*FACTEUR_GRIS_B;
					//float luminosite_texture=texel_ciel.r*FACTEUR_GRIS_R+texel_ciel.g*FACTEUR_GRIS_V+texel_ciel.b*FACTEUR_GRIS_B;
					couleur=couleur_ciel+texel_ciel*(1-luminosite_ciel);
				}
				else
				{
					facteur_brouillard=clamp((dist-brouillard_min)/(zFrustum.y*0.999-brouillard_min),0.,1.);
					couleur=mix(couleur,couleur_ciel,facteur_brouillard);
				}
				if (drapeau_reflet_eau==1) 
				{
					dist = distance_eau;
					facteur_brouillard=clamp((distance_eau-brouillard_min)/(zFrustum.y*0.999-brouillard_min),0.,1.);
					couleur=mix(mix(couleur_eau,couleur,reflexion_eau),couleur_ciel,facteur_brouillard);
				}
				%out.color% =vec4(couleur,1.); 
				%out.depth%=calcule_zDepth(min(dist,zFrustum.y*0.999));
			
		%}
	}
}
