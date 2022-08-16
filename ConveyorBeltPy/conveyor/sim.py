import pybullet as p
import time
import pybullet_data
import random
import shutil
import csv
import os
def start(speed, numObj,freqObj, X, Y, Z, AX, AY, AZ, randAngles, bounceObj, bounceConv, frictionObj, frictionConv, smorLing, smorAng, rolFrict, spinFrict, weight, UI, LOG, logFreq, model, scaling, logName, whatLog, preview):
        
    try:
        p.disconnect()
    except:
        pass
    if UI:    
        physicsClient = p.connect(p.GUI) #connesione a pybullet GRAFICA

    else:
        physicsClient = p.connect(p.DIRECT) #connesione a pybullet SENZA GRAFICA
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)  #setto gravità
    p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=46.4, cameraPitch=-19.6, cameraTargetPosition=[0.16, -0.52, 0.96])  #Visuale
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    planeId = p.loadURDF("plane.urdf") #carico ambiente 
    conveyorPosition = [0,0,0] #posizione del conveyor
    conveyorOrientation = p.getQuaternionFromEuler([0,0,0]) #orientamento del conveyor +0.0000000005549452 destra 6 sinistra
    #Posizione e orientamento nello spazio del grave
    objectOrientation = p.getQuaternionFromEuler([AX,AY,AZ])
    
    objectPosition = [X, Y, Z]
    #Carico il modello del conveyor
    conveyor = p.loadURDF("assets/structure.urdf",conveyorPosition, conveyorOrientation)
    p.changeVisualShape(conveyor,-1,rgbaColor=[0.8,0.8,0.8,1])
    p.changeVisualShape(conveyor,0,rgbaColor=[0.4,0.4,0.4,1.3])
    texture1 = p.loadTexture('assets/rubber.jpg')
    p.changeVisualShape(planeId,-1,textureUniqueId=texture1)
    #Caratteristiche fisiche del conveyor
    p.changeDynamics(1,-1, restitution=bounceConv, mass=200,
                            lateralFriction=frictionConv,rollingFriction=rolFrict, spinningFriction=spinFrict,
                                    linearDamping = smorLing, angularDamping = smorAng)

    #Array in cui salvo i gravi che genero
    gravi = []
    #Array per salvare i dati della simulazione
    log = []
    for i in range(numObj):
        log.append([])
    #Time step della simulazione (meglio non cambiare)
    #p.setTimeStep(0.005)
    try:
        #Variabili per gestione del tempo
        fineSim = 0
        chiusuraSim = 0
        #Inizio simulazione
        for i in range(int(2000000)):
            #Setto la velocità del conveyor
            if (speed>=0.1) | (speed<=-0.1):
                p.resetBaseVelocity(1,[0,speed,0])
                p.stepSimulation()
                p.resetBasePositionAndOrientation(1, conveyorPosition, conveyorOrientation)
            else:
                p.resetBaseVelocity(1,[0,0,0])
                p.stepSimulation()
            time.sleep(1./240.) #Non modificare

            #Ogni tot faccio cadere un nuovo oggetto in base ai dati forniti
            if ((i%(240*freqObj)==0) & (i<(240*freqObj)*numObj)):
                if randAngles:
                    AX = random.randrange(-180,180)
                    AY = random.randrange(-180,180)
                    AZ = random.randrange(-180,180)
                    objectOrientation = p.getQuaternionFromEuler([AX, AY, AZ])
                elif not randAngles:
                    objectOrientation = p.getQuaternionFromEuler([AX,AY,AZ])
                #Genero un nuovo grave e lo salvo nell'array 'gravi'
                gravi.append(p.loadURDF(f"models/{model}",objectPosition,objectOrientation, globalScaling=scaling))
                #Caratteristiche fisiche del grave
                p.changeDynamics(len(gravi)+1,-1, restitution=bounceObj,lateralFriction=frictionObj, 
                                    mass=weight)
                
            #Chiudo simulazione quando tutto è a terra            
            if ((isFinished(gravi)) & (len(gravi) >= numObj)):
                if (fineSim == 0):
                    fineSim = i
                chiusuraSim = i-fineSim
                delayChiusura = 800 #Modificare  se si vuole che la simulazione si chiuda più tardi
                if (chiusuraSim > delayChiusura): 
                    p.disconnect()
                    break

            #Se è una preview faccio vedere solo pochi secondi 
            if ((preview == True) & (i==int(200+240*((2*Z/9.81)**(0.5))))):
                    p.disconnect()
                    break

            #Ottengo i dati necessari per il log
            frequenzaLog = logFreq #Frequenza con cui si salvano i dati nel log
            if (i%int(240*logFreq)==0):
                for e in enumerate(gravi):
                    posizione = str(p.getBasePositionAndOrientation(e[1])[0]).replace(" ", "")
                    orientamento = str(p.getBasePositionAndOrientation(e[1])[1]).replace(" ", "")
                    velocitaLin = str(p.getBaseVelocity(e[1])[0]).replace(" ", "")
                    velocitaAng = str(p.getBaseVelocity(e[1])[1]).replace(" ", "")
                    angoloIniziale = str(f"({AX},{AY},{AZ})")
                    simTime = str(round(i/240, 2))
                    logj = ""
                    header = ""
                    if whatLog[0]:
                        logj += str(e[1])
                        header += 'Oggetto'
                    if whatLog[1]:
                        logj += ";" + simTime
                        header += ';Tempo'
                    if whatLog[2]:
                        logj += ";" + posizione
                        header+= ';Posizione'
                    if whatLog[3]:
                        logj += ";" + orientamento
                        header += ";Orientamento"
                    if whatLog[4]:
                        logj += ";" + velocitaLin
                        header += ";VelocitàLineare"
                    if whatLog[5]:
                        logj += ";" + velocitaAng
                        header += ";VelocitàAngolare"
                    if whatLog[6]:
                        logj += ";" + angoloIniziale
                        header += ";AngoloIniziale"
                    header = [header]
                    #logj = str(e[1]) + ";" + simTime + ";" + posizione + ";" + orientamento +";" + velocitaLin + ";" + velocitaAng + ";" + angoloIniziale 
                    log[e[0]].append(logj)
            
            #Spazio per debug:
            #print(speed)

        #Salvo il log in un file
        if LOG:
            fileName = "LOG/"
            for i in range(len(logName)):
                if logName[i] != ":":
                    fileName = fileName + logName[i]
                else:
                    fileName += "."
            fileName = fileName +".csv"
            
            file = open(fileName, 'w+', newline ='') 
            with file:
                writer = csv.writer(file, delimiter ='-',quotechar =';',quoting=csv.QUOTE_MINIMAL)
                j = 0
                #header = ['Oggetto;Tempo;Posizione;Orientamento;velocitaLineare;velocitaAngolare;Angolo iniziale']
                writer.writerow(header)
                for k in range(numObj): #Per ogni oggetto che è stato generato salvo il log
                    while j < len(log[k]):
                        writer.writerow(log[k][j:j+1])
                        j += 1
                    j = 0
        
    except:
        raise  Exception

#Funzione che determina quando tutti i gravi sono caduti dal conveyor        
def isFinished(gravi):
    for e in enumerate(gravi):
        if p.getBasePositionAndOrientation(e[1])[0][2] > 0.7:
            return False
    return True

#Funziona che crea file URDF da un file STL
def createURDF(filePath, URDFName):
    destination = f"models/{URDFName}.stl" 
    try:
        shutil.copyfile(filePath, destination) #Copio STL nella cartella della simulazione
    except:
        pass
    #shutil.copyfile('models/wheel.urdf', f'models/{URDFName}.urdf')
    #Creo un URDF uguale ad uno default ma cambio il collegamento all'STL
    reading_file = open("assets/conveyor.urdf", "r")
    newURDF = ""
    for line in reading_file:
        stripped_line = line.strip()
        new_line = stripped_line.replace("conveyor", str(URDFName))
        newURDF += new_line +"\n"
    reading_file.close()
    writing_file = open(f'models/{URDFName}.urdf', "w")
    writing_file.write(newURDF)
    writing_file.close()
    return

def modConveyor(scaleX, scaleY):
    #shutil.copyfile('models/wheel.urdf', f'models/{URDFName}.urdf')
    #Creo un URDF uguale ad uno default ma cambio il collegamento all'STL
    reading_file = open("assets/structure_sample.urdf", "r")
    newURDF = ""
    for line in reading_file:
        stripped_line = line.strip()
        new_line = stripped_line.replace("1 1 1", f"{scaleX} {scaleY} 1")
        newURDF += new_line +"\n"
    reading_file.close()
    writing_file = open(f'assets/structure.urdf', "w")
    writing_file.write(newURDF)
    writing_file.close()
    return
#Funziona che trova i modelli disponibili alla simulazione nella cartella
def availableModels():
    STL_list = []
    file_list = os.listdir('models')
    for files in range(len(file_list)):
        if file_list[files][-4:len(file_list[files])] == "urdf":
            STL_list.append(file_list[files])
    return STL_list