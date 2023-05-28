# -*-coding:Utf-8 -*

# ===========================================================

#              - HARFANG® 3D - www.harfang3d.com

#                   Terrain marching

# ===========================================================

import harfang as hg
from math import radians, pow, sqrt
import json
from Car import *

class RenderRect:
    def __init__(self, plus):
        renderer = plus.GetRenderer()

        # create primitive index buffer
        data = hg.BinaryData()
        data.WriteInt16s([0, 1, 2, 0, 2, 3])

        self.indices = renderer.NewBuffer()
        renderer.CreateBuffer(self.indices, data, hg.GpuBufferIndex)

        # create primitive vertex buffer
        self.vertex_layout = hg.VertexLayout()
        self.vertex_layout.AddAttribute(hg.VertexPosition, 3, hg.VertexFloat)
        self.vertex_layout.AddAttribute(hg.VertexUV0, 2, hg.VertexUByte,
                                      True)  # UVs are sent as normalized 8 bit unsigned integer (range [0;255])

        data = hg.BinaryData()
        x, y = 1, 1
        data.WriteFloats([-x, -y, 0])
        data.WriteUInt8s([0, 0])

        data.WriteFloats([-x, y, 0])
        data.WriteUInt8s([0, 255])

        data.WriteFloats([x, y, 0])
        data.WriteUInt8s([255, 255])

        data.WriteFloats([x, -y, 0])
        data.WriteUInt8s([255, 0])

        self.vertex = renderer.NewBuffer()
        renderer.CreateBuffer(self.vertex, data, hg.GpuBufferVertex)

    def draw(self,plus):
        hg.DrawBuffers(plus.GetRenderer(), 6, self.indices, self.vertex, self.vertex_layout)


class RenderToTexture(RenderRect):
    def __init__(self,plus,resolution : hg.Vector2):
        RenderRect.__init__(self,plus)

        renderer = plus.GetRenderer()

        # Shaders:
        self.shader_z_fusion = renderer.LoadShader("assets/shaders/texture_zBuffer.isl")

        # Création des textures de rendu:
        self.texture_rendu_1 = renderer.NewTexture()
        renderer.CreateTexture(self.texture_rendu_1, int(resolution.x), int(resolution.y), hg.TextureRGBA8,
                               hg.TextureNoAA, 0, False)
        self.texture_rendu_1_depth = renderer.NewTexture()
        renderer.CreateTexture(self.texture_rendu_1_depth, int(resolution.x), int(resolution.y), hg.TextureDepth,
                               hg.TextureNoAA, 0, False)

        # Création des frameBuffer objects:
        self.fbo_rendu_1 = renderer.NewRenderTarget()
        renderer.CreateRenderTarget(self.fbo_rendu_1)
        renderer.SetRenderTargetColorTexture(self.fbo_rendu_1, self.texture_rendu_1)
        renderer.SetRenderTargetDepthTexture(self.fbo_rendu_1, self.texture_rendu_1_depth)

        self.projection_matrix_mem = None
        self.view_matrix_mem = None
        self.projection_matrix_ortho = None


    def begin_render(self, plus):
        renderer = plus.GetRenderer()

        renderer.SetWorldMatrix(hg.Matrix4.Identity)
        self.projection_matrix_mem = renderer.GetProjectionMatrix()
        self.view_matrix_mem = renderer.GetViewMatrix()

        self.projection_matrix_ortho = hg.ComputeOrthographicProjectionMatrix(1., 500., 2, hg.Vector2(1, 1))
        renderer.SetProjectionMatrix(self.projection_matrix_ortho)
        renderer.SetViewMatrix(hg.Matrix4.Identity)


    def end_render(self,plus):
        renderer = plus.GetRenderer()
        self.draw(plus)
        renderer.SetProjectionMatrix(self.projection_matrix_mem)
        renderer.SetViewMatrix(self.view_matrix_mem)

    def draw_renderTexture(self,plus):
        self.begin_render(plus)
        renderer.SetShader(self.shader_z_fusion)
        renderer.SetShaderTexture("u_tex", self.texture_rendu_1)
        renderer.SetShaderTexture("u_tex_depth", self.texture_rendu_1_depth)
        self.end_render(plus)

class PhysicTraceTM:
    def __init__(self,pos,normal):
        self.position = pos
        self.normal = normal

    def GetPosition(self):
        return self.position

    def GetNormal(self):
        return self.normal

class TerrainMarching:
    def __init__(self, plus, scene):
        self.lumiere_soleil = scene.GetNode("Sun")
        self.lumiere_ciel = scene.GetNode("SkyLigth")

        self.couleur_horizon = hg.Color(255. / 255. * 0.75, 221. / 255. * 0.75, 199 / 255. * 0.75, 1.)
        # self.couleur_zenith=hg.Color(255/255.,252./255.,171./255.,1.)
        self.couleur_zenith = hg.Color(70. / 255. / 2, 150. / 255. / 2, 255. / 255. / 2, 1.)

        self.couleur_ambiante = hg.Color(78. / 255., 119. / 255., 107. / 255., 1.)
        # self.couleur_ambiante=hg.Color(1,0,0,1)

        renderer = plus.GetRenderer()
        # -------------- Init le shader de rendu de terrain:
        self.shader_terrain = renderer.LoadShader("assets/shaders/terrain_marching_montagnes.isl")
        self.bitmap_terrain1 = hg.Picture()
        self.bitmap_terrain2 = hg.Picture()
        self.bitmap_terrain3 = hg.Picture()
        hg.LoadPicture(self.bitmap_terrain1,"assets/textures/terrain_1.png")
        hg.LoadPicture(self.bitmap_terrain2,"assets/textures/bruit_2048.png")
        hg.LoadPicture(self.bitmap_terrain3,"assets/textures/terrain_2048.png")

        self.texture_terrain1 = renderer.NewTexture("terrain_1",self.bitmap_terrain1)
        self.texture_terrain2 = renderer.NewTexture("terrain_2",self.bitmap_terrain2)
        self.texture_terrain3 = renderer.NewTexture("terrain_3",self.bitmap_terrain3)

        self.texture_cosmos = renderer.LoadTexture("assets/textures/cosmos_b.png")

        self.facteur_echelle_terrain_l1 = hg.Vector2(20000, 20000)
        self.facteur_echelle_terrain_l2 = hg.Vector2(1000, 1000)
        self.facteur_echelle_terrain_l3 = hg.Vector2(100, 100)

        self.amplitude_l1 = 1000
        self.amplitude_l2 = 90
        self.amplitude_l3 = 1.5
        self.intensite_ambiante = 0.2

        self.offset_terrain = hg.Vector3(0, -50, 0)

        self.facteur_precision_distance = 1.01
        self.couleur_neige = hg.Color(0.91 * 0.75, 0.91 * 0.75, 1 * 0.75)
        self.couleur_cristaux = hg.Color(133 / 255, 225 / 255, 181 / 255, 1)
        self.couleur_eau = hg.Color(117. / 255., 219. / 255., 211. / 255.)
        self.altitude_eau = 0
        self.reflexion_eau = 0.5
        self.intensite_cristaux = 3

        self.couleur_mineral_1 = hg.Color(0.3,0.3,0.3)
        self.couleur_mineral_2 = hg.Color(122./255.,105./255.,95./255.)
        self.speculaire_mineral = hg.Color(0.8,0.8,0.8)
        self.speculaire_neige = hg.Color(1,1,1)
        self.brillance_neige = 30
        self.brillance_mineral = 90
        self.mineral_fading = hg.Vector2 (100,500)

    def get_normal(self,pos:hg.Vector3):
        f=1.5
        return hg.Vector3(self.get_altitude(hg.Vector3(pos.x-f,0,pos.z))-self.get_altitude(hg.Vector3(pos.x+f,0,pos.z)),
                          2*f,
                          self.get_altitude(hg.Vector3(pos.x,0,pos.z-f))-self.get_altitude(hg.Vector3(pos.x,0,pos.z+f))).Normalized()

    def intersect(self,pos:hg.Vector3, dir:hg.Vector3):
        dist=0.01
        for i in range(128):
            p=pos+dir*dist
            alt=p.y-self.get_altitude(p)
            if alt < 0.002*dist:
                break
            dist+=0.3*alt
        return dist

    def raycast(self,pos,dir,dist_max):
        dist=self.intersect(pos,dir)
        if dist>dist_max:
            return False,None
        else:
            return True,PhysicTraceTM(pos+dir*dist,self.get_normal(pos))

    def get_altitude(self,pos:hg.Vector3):
        pos -= self.offset_terrain
        pos2=hg.Vector2(pos.x,pos.z)
        a=self.get_pixel_bilinear(self.bitmap_terrain1,pos2/self.facteur_echelle_terrain_l1).x
        b=self.get_pixel_bilinear(self.bitmap_terrain2,pos2/self.facteur_echelle_terrain_l2).x
        c=self.get_pixel_bilinear(self.bitmap_terrain3,pos2/self.facteur_echelle_terrain_l3).x
        return pow(a, 5.) * self.amplitude_l1 + pow(b, 4.) * self.amplitude_l2 + c * self.amplitude_l3 + self.offset_terrain.y

    def get_pixel_bilinear(self, picture:hg.Picture, pos:hg.Vector2 ):
        w=picture.GetWidth()
        h=picture.GetHeight()
        x=(pos.x*w -0.5) % w
        y=(pos.y*h -0.5) % h
        xi=int(x)
        yi=int(y)
        xf=x-xi
        yf=y-yi
        xi1=(xi+1) % w
        yi1=(yi+1) % h
        c1=picture.GetPixelRGBA(xi,yi)
        c2=picture.GetPixelRGBA(xi1,yi)
        c3=picture.GetPixelRGBA(xi,yi1)
        c4=picture.GetPixelRGBA(xi1,yi1)
        c12=c1*(1-xf)+c2*xf
        c34=c3*(1-xf)+c4*xf
        c=c12*(1-yf)+c34*yf
        return c

    def load_json_script(self,file_name="assets/scripts/terrain_parameters.json"):
        json_script = hg.GetFilesystem().FileToString(file_name)
        if json_script != "":
            script_parameters = json.loads(json_script)
            self.facteur_echelle_terrain_l1 = list_to_vec2(script_parameters["map1_scale"])
            self.facteur_echelle_terrain_l2 = list_to_vec2(script_parameters["map2_scale"])
            self.facteur_echelle_terrain_l3 = list_to_vec2(script_parameters["map3_scale"])
            self.amplitude_l1 = script_parameters["map1_amplitude"]
            self.amplitude_l2 = script_parameters["map2_amplitude"]
            self.amplitude_l3 = script_parameters["map3_amplitude"]
            self.altitude_eau = script_parameters["water_altitude"]
            self.reflexion_eau = script_parameters["water_reflexion"]
            self.offset_terrain = list_to_vec3(script_parameters["offset_terrain"])
            self.couleur_mineral_1 = list_to_color(script_parameters["mineral_1_color"])
            self.couleur_mineral_2 = list_to_color(script_parameters["mineral_2_color"])
            self.mineral_fading = list_to_vec2(script_parameters["mineral_fading"])
            self.speculaire_neige = list_to_color(script_parameters["speculaire_neige"])
            self.speculaire_mineral = list_to_color(script_parameters["speculaire_mineral"])
            self.brillance_mineral = script_parameters["brillance_mineral"]
            self.brillance_neige = script_parameters["brillance_neige"]
            self.couleur_eau = list_to_color(script_parameters["water_color"])
            self.couleur_zenith = list_to_color(script_parameters["zenith_color"])
            self.intensite_ambiante = script_parameters["ambiant_intensity"]
            self.couleur_horizon = list_to_color(script_parameters["horizon_color"])
            self.couleur_neige = list_to_color(script_parameters["snow_color"])
            self.couleur_cristaux = list_to_color(script_parameters["gem_color"])
            self.lumiere_soleil.GetLight().SetDiffuseColor(list_to_color(script_parameters["sunlight_color"]))
            self.lumiere_ciel.GetLight().SetDiffuseColor(list_to_color(script_parameters["skylight_color"]))

    def save_json_script(self,output_filename = "assets/scripts/terrain_parameters.json"):
        script_parameters = {"map1_scale": vec2_to_list(self.facteur_echelle_terrain_l1),
                             "map2_scale": vec2_to_list(self.facteur_echelle_terrain_l2),
                             "map3_scale": vec2_to_list(self.facteur_echelle_terrain_l3),
                             "map1_amplitude": self.amplitude_l1,
                             "map2_amplitude": self.amplitude_l2, "map3_amplitude": self.amplitude_l3,
                             "water_altitude": self.altitude_eau,
                             "water_reflexion": self.reflexion_eau,
                             "offset_terrain": vec3_to_list(self.offset_terrain),
                             "mineral_1_color": color_to_list(self.couleur_mineral_1),
                             "mineral_2_color": color_to_list(self.couleur_mineral_2),
                             "speculaire_neige": color_to_list(self.speculaire_neige),
                             "brillance_mineral": self.brillance_mineral,
                             "speculaire_mineral": color_to_list(self.speculaire_mineral),
                             "brillance_neige": self.brillance_neige,
                             "mineral_fading": vec2_to_list(self.mineral_fading),
                             "water_color": color_to_list(self.couleur_eau),
                             "zenith_color": color_to_list(self.couleur_zenith),
                             "ambiant_intensity": self.intensite_ambiante,
                             "horizon_color": color_to_list(self.couleur_horizon),
                             "snow_color": color_to_list(self.couleur_neige),
                             "gem_color": color_to_list(self.couleur_cristaux),
                             "sunlight_color": color_to_list(self.lumiere_soleil.GetLight().GetDiffuseColor()),
                             "skylight_color": color_to_list(self.lumiere_ciel.GetLight().GetDiffuseColor())
                             }
        json_script = json.dumps(script_parameters, indent=4)
        return hg.GetFilesystem().StringToFile(output_filename, json_script)

    def update_shader(self,plus , scene, resolution, dts):
        camera = scene.GetNode("Camera")
        renderer=plus.GetRenderer()
        renderer.EnableDepthTest(True)
        renderer.EnableDepthWrite(True)
        renderer.EnableBlending(False)

        #renderer.SetViewport(hg.fRect(0, 0, resolution.x, resolution.y))  # fit viewport to window dimensions
        #renderer.Clear(hg.Color.Black, 1, hg.GpuRenderer.ClearDepth)

        renderer.SetShader(self.shader_terrain)
        renderer.SetShaderTexture("texture_terrain", self.texture_terrain1)
        renderer.SetShaderTexture("texture_terrain2", self.texture_terrain2)
        renderer.SetShaderTexture("texture_terrain3", self.texture_terrain3)
        renderer.SetShaderTexture("texture_ciel", self.texture_cosmos)
        renderer.SetShaderFloat("ratio_ecran",resolution.x/resolution.y)
        renderer.SetShaderFloat("distanceFocale",camera.GetCamera().GetZoomFactor())
        cam=camera.GetTransform()
        camPos=cam.GetPreviousWorld().GetTranslation()
        renderer.SetShaderFloat3("obs_pos",camPos.x,camPos.y,camPos.z)
        renderer.SetShaderMatrix3("obs_mat_normale",cam.GetPreviousWorld().GetRotationMatrix())
        renderer.SetShaderFloat2("facteur_echelle_terrain1",1./self.facteur_echelle_terrain_l1.x,1./self.facteur_echelle_terrain_l1.y)
        renderer.SetShaderFloat2("facteur_echelle_terrain2",1./self.facteur_echelle_terrain_l2.x,1./self.facteur_echelle_terrain_l2.y)
        renderer.SetShaderFloat2("facteur_echelle_terrain3",1./self.facteur_echelle_terrain_l3.x,1./self.facteur_echelle_terrain_l3.y)
        renderer.SetShaderFloat("amplitude_terrain1",self.amplitude_l1)
        renderer.SetShaderFloat("amplitude_terrain2",self.amplitude_l2)
        renderer.SetShaderFloat("amplitude_terrain3",self.amplitude_l3)
        renderer.SetShaderFloat3("offset_terrain",self.offset_terrain.x,self.offset_terrain.y,self.offset_terrain.z)
        renderer.SetShaderFloat("altitude_eau",self.altitude_eau)
        renderer.SetShaderFloat("reflexion_eau",self.reflexion_eau)
        renderer.SetShaderFloat("intensite_ambiante",self.intensite_ambiante)
        renderer.SetShaderFloat3("couleur_mineral1",self.couleur_mineral_1.r,self.couleur_mineral_1.g,self.couleur_mineral_1.b)
        renderer.SetShaderFloat3("couleur_mineral2",self.couleur_mineral_2.r,self.couleur_mineral_2.g,self.couleur_mineral_2.b)
        renderer.SetShaderFloat2("mineral_fading", self.mineral_fading.x, self.mineral_fading.y)
        renderer.SetShaderFloat3("speculaire_mineral",self.speculaire_mineral.r,self.speculaire_mineral.g,self.speculaire_mineral.b)
        renderer.SetShaderFloat3("speculaire_neige",self.speculaire_neige.r,self.speculaire_neige.g,self.speculaire_neige.b)
        renderer.SetShaderFloat("brillance_neige", self.brillance_neige)
        renderer.SetShaderFloat("brillance_mineral", self.brillance_mineral)
        renderer.SetShaderFloat3("couleur_zenith",self.couleur_zenith.r,self.couleur_zenith.g,self.couleur_zenith.b)
        renderer.SetShaderFloat3("couleur_horizon",self.couleur_horizon.r,self.couleur_horizon.g,self.couleur_horizon.b)
        renderer.SetShaderFloat3("couleur_neige",self.couleur_neige.r,self.couleur_neige.g,self.couleur_neige.b)
        renderer.SetShaderFloat3("couleur_eau",self.couleur_eau.r,self.couleur_eau.g,self.couleur_eau.b)
        renderer.SetShaderFloat3("couleur_cristaux",self.couleur_cristaux.r*self.intensite_cristaux,\
                                   self.couleur_cristaux.g*self.intensite_cristaux,self.couleur_cristaux.b*self.intensite_cristaux)

        l_dir=self.lumiere_soleil.GetTransform().GetWorld().GetRotationMatrix().GetZ()

        renderer.SetShaderFloat3("l1_direction",l_dir.x,l_dir.y,l_dir.z)
        l_couleur=self.lumiere_soleil.GetLight().GetDiffuseColor()
        renderer.SetShaderFloat3("l1_couleur",l_couleur.r,l_couleur.g,l_couleur.b)

        l_dir=self.lumiere_ciel.GetTransform().GetWorld().GetRotationMatrix().GetZ()
        renderer.SetShaderFloat3("l2_direction",l_dir.x,l_dir.y,l_dir.z)
        l_couleur=self.lumiere_ciel.GetLight().GetDiffuseColor()*self.lumiere_ciel.GetLight().GetDiffuseIntensity()
        renderer.SetShaderFloat3("l2_couleur",l_couleur.r,l_couleur.g,l_couleur.b)
        renderer.SetShaderFloat("temps",dts)

        renderer.SetShaderFloat2("zFrustum",camera.GetCamera().GetZNear(),camera.GetCamera().GetZFar())


# =============================================================================================

#       Fonctions

# =============================================================================================

def list_to_color(c: list):
    return hg.Color(c[0], c[1], c[2], c[3])


def color_to_list(c: hg.Color):
    return [c.r, c.g, c.b, c.a]

def list_to_vec2(v: list):
    return hg.Vector2(v[0], v[1])

def vec2_to_list(v: hg.Vector2):
    return [v.x, v.y]

def list_to_vec3(v: list):
    return hg.Vector3(v[0], v[1],v[2])

def vec3_to_list(v: hg.Vector3):
    return [v.x, v.y, v.z]

def init_scene(plus):
    scene = plus.NewScene()
    camera = plus.AddCamera(scene,hg.Matrix4.TranslationMatrix(hg.Vector3(0, 0, -10)))
    camera.SetName("Camera")
    camera.GetCamera().SetZNear(1.)
    camera.GetCamera().SetZFar(10000)

    plus.LoadScene(scene, "assets/car_big_wheeler/car_big_wheeler.scn")

    pos=hg.Vector3(2726,28,-1915.98046875)
    pos=hg.Vector3(0,0,0)
    cube = plus.AddPhysicCube(scene,hg.Matrix4.TranslationMatrix(pos),50,10,50,0)
    cube[1].SetType(hg.RigidBodyKinematic)

    init_lights(plus, scene)
    fps = hg.FPSController(0,500,-10)

    #while not scene.IsReady():                      # Wait until scene is ready
    for i in range(256):
        plus.UpdateScene(scene, plus.UpdateClock())

    car = Car("Kubolid", plus, scene, hg.Vector3(0,0,0)) # mosesli

    return scene, fps, cube,car


def init_lights(plus, scene):
    # Main light:
    ligth_sun = plus.AddLight(scene, hg.Matrix4.RotationMatrix(hg.Vector3(radians(15),radians(-45), 0)),
                            hg.LightModelLinear)
    ligth_sun.SetName("Sun")
    ligth_sun.GetLight().SetDiffuseColor(hg.Color(255. / 255., 255. / 255., 255. / 255., 1.))

    ligth_sun.GetLight().SetShadow(hg.LightShadowMap)  # Active les ombres portées
    ligth_sun.GetLight().SetShadowRange(100)

    ligth_sun.GetLight().SetDiffuseIntensity(1.)
    ligth_sun.GetLight().SetSpecularIntensity(1.)

    # Sky ligth:
    ligth_sky = plus.AddLight(scene, hg.Matrix4.RotationMatrix(hg.Vector3(radians(54), radians(135), 0)),
                            hg.LightModelLinear)
    ligth_sky.SetName("SkyLigth")
    ligth_sky.GetLight().SetDiffuseColor(hg.Color(103. / 255., 157. / 255., 141. / 255., 1.))
    ligth_sky.GetLight().SetDiffuseIntensity(0.9)


def init_terrain(plus,scene):
    terrain = TerrainMarching(plus,scene)
    terrain.load_json_script()
    return terrain


def gui_interface(terrain : TerrainMarching, scene, fps):

    camera = scene.GetNode("Camera")

    l1_color = terrain.lumiere_soleil.GetLight().GetDiffuseColor()
    l2_color = terrain.lumiere_ciel.GetLight().GetDiffuseColor()

    if hg.ImGuiBegin("Settings"):
        #f = hg.ImGuiInputVector2("Map 1 scale",terrain.facteur_echelle_terrain_l1,1)

        if hg.ImGuiButton("Load parameters"):
            terrain.load_json_script()
        hg.ImGuiSameLine()
        if hg.ImGuiButton("Save parameters"):
            terrain.save_json_script()
        if hg.ImGuiButton("Load camera"):
            load_fps_matrix(fps)
        hg.ImGuiSameLine()
        if hg.ImGuiButton("Save camera"):
            save_json_matrix(camera.GetTransform().GetPosition(),camera.GetTransform().GetRotation())

        if hg.ImGuiButton("Load car position"):
            load_car_start_position(car,terrain)
        hg.ImGuiSameLine()
        if hg.ImGuiButton("Save car position"):
            save_json_matrix(car.get_parent_node().GetTransform().GetPosition(),hg.Vector3(0,0,0),"assets/scripts/car_position.json")

        f,c = hg.ImGuiColorEdit("Water color", terrain.couleur_eau)
        if f: terrain.couleur_eau = c
        f, c = hg.ImGuiColorEdit("Snow color", terrain.couleur_neige)
        if f: terrain.couleur_neige = c
        f, c = hg.ImGuiColorEdit("Mineral 1 color", terrain.couleur_mineral_1)
        if f: terrain.couleur_mineral_1 = c
        f, c = hg.ImGuiColorEdit("Mineral 2 color", terrain.couleur_mineral_2)
        if f: terrain.couleur_mineral_2 = c
        hg.ImGuiText("W/X mineral fading min: " + str(terrain.mineral_fading.x))
        hg.ImGuiText("C/V mineral fading max: " + str(terrain.mineral_fading.y))
        f, c = hg.ImGuiColorEdit("Mineral_specular", terrain.speculaire_mineral)
        if f: terrain.speculaire_mineral = c
        f, c = hg.ImGuiColorEdit("Snow specular", terrain.speculaire_neige)
        if f: terrain.speculaire_neige = c
        hg.ImGuiText("5/6 Mineral shininess: " + str(terrain.brillance_mineral))
        hg.ImGuiText("7/8 Snow shininess: " + str(terrain.brillance_neige))
        f, c = hg.ImGuiColorEdit("Horizon color", terrain.couleur_horizon)
        if f: terrain.couleur_horizon = c
        f, c = hg.ImGuiColorEdit("Zenith color", terrain.couleur_zenith)
        if f: terrain.couleur_zenith = c
        hg.ImGuiText("9/0 Ambient intensity: " + str(terrain.intensite_ambiante))
        f, c = hg.ImGuiColorEdit("Gem color", terrain.couleur_cristaux)
        if f: terrain.couleur_cristaux = c

        f, c = hg.ImGuiColorEdit("Sunlight color", l1_color)
        if f:
            l1_color = hg.Color(c)

        f, c2 = hg.ImGuiColorEdit("Skylight color", l2_color)
        if f:
            l2_color = hg.Color(c2)

        hg.ImGuiText("A/Q facteur_echelle_terrain_l1.x: " + str(terrain.facteur_echelle_terrain_l1.x))
        hg.ImGuiText("Z/S facteur_echelle_terrain_l1.y: " + str(terrain.facteur_echelle_terrain_l1.y))
        hg.ImGuiText("E/D facteur_echelle_terrain_l2.x: " + str(terrain.facteur_echelle_terrain_l2.x))
        hg.ImGuiText("R/F facteur_echelle_terrain_l2.y: " + str(terrain.facteur_echelle_terrain_l2.y))
        hg.ImGuiText("T/G facteur_echelle_terrain_l3.x: " + str(terrain.facteur_echelle_terrain_l3.x))
        hg.ImGuiText("Y/H facteur_echelle_terrain_l3.y: " + str(terrain.facteur_echelle_terrain_l3.y))
        hg.ImGuiText("U/J amplitude_l1: " + str(terrain.amplitude_l1))
        hg.ImGuiText("I/K amplitude_l2: " + str(terrain.amplitude_l2))
        hg.ImGuiText("O/L amplitude_l3: " + str(terrain.amplitude_l3))
        hg.ImGuiText("P/M facteur_precision_distance: " + str(terrain.facteur_precision_distance))
        hg.ImGuiText("1/2 altitude_eau: " + str(terrain.altitude_eau))
        hg.ImGuiText("3/4 reflexion_eau: " + str(terrain.reflexion_eau))
        hg.ImGuiText("Ctrl + 5/6 offset terrain X: " + str(terrain.offset_terrain.x))
        hg.ImGuiText("Ctrl + 7/8 offset terrain Y: " + str(terrain.offset_terrain.y))
        hg.ImGuiText("Ctrl + 9/0 offset terrain Z: " + str(terrain.offset_terrain.z))
        hg.ImGuiText("CTRL+S sauve paramètres")
        hg.ImGuiText("CTRL+L charge paramètres")

    hg.ImGuiEnd()

    return l1_color,l2_color

def load_car_start_position(car,terrain):
    pos, rot = load_json_matrix("assets/scripts/car_position.json")
    alt=terrain.get_altitude(pos)
    pos.y=alt+2
    car.reset(pos)

def load_fps_matrix(fps):
    pos, rot = load_json_matrix()
    if pos is not None and rot is not None:
        fps.Reset(pos, rot)

def load_json_matrix(file_name="assets/scripts/camera_position.json"):
    json_script = hg.GetFilesystem().FileToString(file_name)
    if json_script != "":
        pos = hg.Vector3()
        rot = hg.Vector3()
        script_parameters = json.loads(json_script)
        pos.x=script_parameters["x"]
        pos.y=script_parameters["y"]
        pos.z=script_parameters["z"]
        rot.x=script_parameters["rot_x"]
        rot.y=script_parameters["rot_y"]
        rot.z=script_parameters["rot_z"]
        return pos,rot
    return None,None

def save_json_matrix(pos : hg.Vector3, rot:hg.Vector3,output_filename = "assets/scripts/camera_position.json"):
    script_parameters = {"x": pos.x,
                         "y": pos.y,
                         "z": pos.z,
                         "rot_x": rot.x,
                         "rot_y": rot.y,
                         "rot_z": rot.z}
    json_script = json.dumps(script_parameters, indent=4)
    return hg.GetFilesystem().StringToFile(output_filename, json_script)

def edition_clavier(terrain):
    if plus.KeyDown(hg.KeyLAlt):
        f = 100
    else:
        f = 1

    if not plus.KeyDown(hg.KeyLCtrl):

        if plus.KeyDown(hg.KeyA):
            terrain.facteur_echelle_terrain_l1.x += 10
        elif plus.KeyDown(hg.KeyQ):
            terrain.facteur_echelle_terrain_l1.x -= 10
            if terrain.facteur_echelle_terrain_l1.x < 10:
                terrain.facteur_echelle_terrain_l1.x = 10

        if plus.KeyDown(hg.KeyZ):
            terrain.facteur_echelle_terrain_l1.y += 10
        elif plus.KeyDown(hg.KeyS):
            terrain.facteur_echelle_terrain_l1.y -= 10
            if terrain.facteur_echelle_terrain_l1.y < 10:
                terrain.facteur_echelle_terrain_l1.y = 10

        if plus.KeyDown(hg.KeyE):
            terrain.facteur_echelle_terrain_l2.x += 10
        elif plus.KeyDown(hg.KeyD):
            terrain.facteur_echelle_terrain_l2.x -= 10
            if terrain.facteur_echelle_terrain_l2.x < 10:
                terrain.facteur_echelle_terrain_l2.x = 10

        if plus.KeyDown(hg.KeyR):
            terrain.facteur_echelle_terrain_l2.y += 10
        elif plus.KeyDown(hg.KeyF):
            terrain.facteur_echelle_terrain_l2.y -= 10
            if terrain.facteur_echelle_terrain_l2.y < 10:
                terrain.facteur_echelle_terrain_l2.y = 10

        if plus.KeyDown(hg.KeyT):
            terrain.facteur_echelle_terrain_l3.x += 0.1
        elif plus.KeyDown(hg.KeyG):
            terrain.facteur_echelle_terrain_l3.x -= 0.1
            if terrain.facteur_echelle_terrain_l3.x < 0.1:
                terrain.facteur_echelle_terrain_l3.x = 0.1

        if plus.KeyDown(hg.KeyY):
            terrain.facteur_echelle_terrain_l3.y += 0.1
        elif plus.KeyDown(hg.KeyH):
            terrain.facteur_echelle_terrain_l3.y -= 0.1
            if terrain.facteur_echelle_terrain_l3.y < 0.1:
                terrain.facteur_echelle_terrain_l3.y = 0.1

        elif plus.KeyDown(hg.KeyU):
            terrain.amplitude_l1 += 1
        elif plus.KeyDown(hg.KeyJ):
            terrain.amplitude_l1 -= 1
            # if terrain.amplitude_l1<100:
            # terrain.amplitude_l1=100

        elif plus.KeyDown(hg.KeyI):
            terrain.amplitude_l2 += 10
        elif plus.KeyDown(hg.KeyK):
            terrain.amplitude_l2 -= 10
            # if terrain.amplitude_l2<10:
            # terrain.amplitude_l2=10

        elif plus.KeyDown(hg.KeyO):
            terrain.amplitude_l3 += 0.05
        elif plus.KeyDown(hg.KeyL):
            terrain.amplitude_l3 -= 0.05
            # if terrain.amplitude_l3<0.05:
            # terrain.amplitude_l3=0.05


        elif plus.KeyDown(hg.KeyP):
            terrain.facteur_precision_distance += 0.001
        elif plus.KeyDown(hg.KeyM):
            terrain.facteur_precision_distance -= 0.001
            if terrain.facteur_precision_distance < 1.001:
                terrain.facteur_precision_distance = 1.001

        elif plus.KeyDown(hg.KeyNumpad2):
            terrain.altitude_eau += 1
        elif plus.KeyDown(hg.KeyNumpad1):
            terrain.altitude_eau -= 1

        elif plus.KeyDown(hg.KeyNumpad4):
            terrain.reflexion_eau += 0.01
        elif plus.KeyDown(hg.KeyNumpad3):
            terrain.reflexion_eau -= 0.01

        terrain.reflexion_eau = max(min(1,terrain.reflexion_eau),0)

        if plus.KeyDown(hg.KeyNumpad0):
            terrain.intensite_ambiante += 0.01
            terrain.intensite_ambiante = min(terrain.intensite_ambiante,1)
        elif plus.KeyDown(hg.KeyNumpad9):
            terrain.intensite_ambiante -= 0.01
            terrain.intensite_ambiante = max(terrain.intensite_ambiante, 0)

        if plus.KeyDown(hg.KeyX):
            terrain.mineral_fading.x += 1
        elif plus.KeyDown(hg.KeyW):
            terrain.mineral_fading.x -= 1
            terrain.mineral_fading.x = min(terrain.mineral_fading.y,terrain.mineral_fading.x)

        if plus.KeyDown(hg.KeyV):
            terrain.mineral_fading.y += 1
        elif plus.KeyDown(hg.KeyC):
            terrain.mineral_fading.y -= 1
            terrain.mineral_fading.y = max(terrain.mineral_fading.y,terrain.mineral_fading.x)

        if plus.KeyDown(hg.KeyNumpad6):
            terrain.brillance_mineral += 1
        elif plus.KeyDown(hg.KeyNumpad5):
            terrain.brillance_mineral -= 1
            terrain.brillance_mineral = max(terrain.brillance_mineral,1)

        if plus.KeyDown(hg.KeyNumpad8):
            terrain.brillance_neige += 1
        elif plus.KeyDown(hg.KeyNumpad7):
            terrain.brillance_neige -= 1
            terrain.brillance_neige = max(terrain.brillance_neige,1)

        sun = scene.GetNode("Sun")
        rot = sun.GetTransform().GetRotation()
        if plus.KeyDown(hg.KeyF1):
            rot.x += radians(1)
        elif plus.KeyDown(hg.KeyF2):
            rot.x -= radians(1)
        if plus.KeyDown(hg.KeyF3):
            rot.y += radians(1)
        elif plus.KeyDown(hg.KeyF4):
            rot.y -= radians(1)
        sun.GetTransform().SetRotation(rot)


    elif plus.KeyDown(hg.KeyLCtrl):
        if plus.KeyPress(hg.KeyS):
            terrain.save_json_script()
        elif plus.KeyPress(hg.KeyL):
            terrain.load_parameters()

        if plus.KeyDown(hg.KeyNumpad6):
            terrain.offset_terrain.x += 0.1 * f
        elif plus.KeyDown(hg.KeyNumpad5):
            terrain.offset_terrain.x -= 0.1 * f

        elif plus.KeyDown(hg.KeyNumpad8):
            terrain.offset_terrain.y += 0.01 * f
        elif plus.KeyDown(hg.KeyNumpad7):
            terrain.offset_terrain.y -= 0.01 * f

        elif plus.KeyDown(hg.KeyNumpad0):
            terrain.offset_terrain.z += 0.1 * f
        elif plus.KeyDown(hg.KeyNumpad9):
            terrain.offset_terrain.z -= 0.1 * f



def update_view_fps(plus, scene, terrain, fps, delta_t):
    pos = fps.GetPos()
    altitude = terrain.get_altitude(pos)
    pos = fps.GetPos()
    pos.y = altitude + 20
    rot = fps.GetRot()
    fps.Reset(pos, rot)

    camera = scene.GetNode("Camera")
    fps.UpdateAndApplyToNode(camera,delta_t)


def car_control(plus, car, terrain, dts):
    if plus.KeyDown(hg.KeyUp):
        car.accelerate(10)
    if plus.KeyDown(hg.KeyDown):
        car.accelerate(-10)
    if plus.KeyDown(hg.KeySpace):
        car.brake(10)
    if plus.KeyDown(hg.KeyLeft):
        car.turn(-150 * dts)
    if plus.KeyDown(hg.KeyRight):
        car.turn(150 * dts)
    if plus.KeyPress (hg.KeyBackspace):
        pos=car.get_position()
        alt=terrain.get_altitude(pos)
        pos.y=alt+2
        car.reset(pos)

# ==================================================================================================

#                                   Program start here

# ==================================================================================================

# Display settings
resolution = hg.Vector2(1920, 1080)
antialiasing = 1
screenMode = hg.Windowed

# System setup
plus = hg.GetPlus()
hg.LoadPlugins()
plus.Mount("./")

# Run display
plus.RenderInit(int(resolution.x), int(resolution.y), antialiasing, screenMode)
plus.SetBlend2D(hg.BlendAlpha)

# Setup dashboard:
scene, fps, cube, car = init_scene(plus)
render_to_texture = RenderToTexture(plus,resolution)
terrain = init_terrain(plus,scene)
load_fps_matrix(fps)

camera=scene.GetNode("Camera")
pos,rot=load_json_matrix()
camera.GetTransform().SetPosition(pos)
camera.GetTransform().SetRotation(rot)

load_car_start_position(car,terrain)

#while not scene.IsReady():  # Wait until scene is ready
plus.UpdateScene(scene, plus.UpdateClock())
plus.UpdateScene(scene, plus.UpdateClock())

setup_camera_follow(car.get_parent_node(), car.get_parent_node().GetTransform().GetPosition())

#camera=scene.GetNode("Camera")
#pos = camera.GetTransform().GetPosition()
#pos.y+=5
#cube.GetTransform().SetPosition(pos)

# -----------------------------------------------
#                   Main loop
# -----------------------------------------------
delta_t=0

while not plus.IsAppEnded():

    delta_t = plus.UpdateClock()
    dts = hg.time_to_sec_f(delta_t)

    # Editor:
    edition_clavier(terrain)
    l1_color, l2_color = gui_interface(terrain, scene, fps)

    # 3D anim:
    car_control(plus,car,terrain,dts)
    car.update_kinetic(scene, terrain, dts)
    #update_view_fps(plus, scene, terrain, fps, delta_t)
    update_camera_follow(camera, scene,terrain, dts)
    # Scene 3d:

    renderer = plus.GetRenderer()
    renderer.SetRenderTarget(render_to_texture.fbo_rendu_1)
    plus.UpdateScene(scene)

    # Raymarching
    renderer.ClearRenderTarget()
    renderer.Clear(hg.Color.Black)  # red
    render_to_texture.begin_render(plus)
    terrain.update_shader(plus, scene, resolution, hg.time_to_sec_f(plus.GetClock()))
    render_to_texture.end_render(plus)

    # Fusion:
    render_to_texture.draw_renderTexture(plus)

    plus.Flip()
    plus.EndFrame()

    terrain.lumiere_soleil.GetLight().SetDiffuseColor(l1_color)
    terrain.lumiere_ciel.GetLight().SetDiffuseColor(l2_color)

plus.RenderUninit()