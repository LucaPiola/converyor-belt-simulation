from logging import exception
import PySimpleGUI as sg
import pybullet as p
import time
import pybullet_data
import random
from datetime import datetime
import threading
import multiprocessing

from sim import modConveyor, start, createURDF, availableModels
sg.theme('DarkTeal1')   
#SCHEDA 'SIMULAZIONE'
import os
tab_sim =  [[sg.Text('', size=(24, 1)),sg.Text('', size=(24, 1))],
            [sg.T('IMPOSTAZIONI SIMULAZIONE', size=(30, 1), justification='left',font=('Any', 15, 'bold'))],
            [sg.Text('  Numero oggetti da generare:', size=(24, 1)),  sg.Slider(range=(1, 15),key='-numObj', default_value=2, size=(15,10),orientation='horizontal',visible=True),                                                         sg.Text("[1,15]", key='-limitNum')],
            
            [sg.Text('  Tempo tra generazioni: (sec)', size=(24, 1)), sg.InputText("1", key='-freq',size=(4, 1), justification='center')],
            [sg.Text('', size=(24, 2))],

            [sg.T('IMPOSTAZIONI OGGETTO', size=(30, 1), justification='left',font=('Any', 15,'bold'))],
            [sg.Text('  Coordinate caduta oggetto:', size=(26, 1)), sg.Text('X', size=(3, 1)), sg.InputText("0",key='-x', size=(4, 1), justification='center'),
                                                                  sg.Text('Y', size=(3, 1)), sg.InputText("-0.8", key='-y',size=(4, 1), justification='center'),
                                                                  sg.Text('Z', size=(3, 1)), sg.InputText("2", key='-z',size=(4, 1), justification='center')],
            
            [sg.Text('  Inclinazione oggetto iniziale:', key='-testo1', size=(26, 1)), sg.Text('AX',key='-testo2',size=(3, 1)), sg.InputText("0", key='-ax', size=(4, 1), justification='center'),
                                                                  sg.Text('AY',key='-testo3',size=(3, 1)), sg.InputText("0", key='-ay', size=(4, 1), justification='center'),
                                                                  sg.Text('AZ', key='-testo4',size=(3, 1)), sg.InputText("0", key='-az',size=(4, 1), justification='center'),
                                                                  ],
            [sg.Text("  Attiva Angoli casuali ad ogni generazione: "), sg.Checkbox("", key='-randAngles', enable_events=True)],
            
            [sg.Text('  Scala oggetto: ', size=(24, 1)), sg.InputText("1",key='-scale', size=(4, 1), justification='center', enable_events=True), sg.Button('Anteprima', key='-preview')],
            [sg.Text('', size=(24, 2))],
            [sg.T('MODIFICA CONVEYOR', size=(30, 1), justification='left',font=('Any', 15, 'bold'))],
            [sg.Text('  Scala X: ', size=(10, 1)), sg.InputText("1",key='-scaleConvX', size=(4, 1), justification='center', enable_events=True),
            sg.Text('  Scala X: ', size=(10, 1)), sg.InputText("1",key='-scaleConvY', size=(4, 1), justification='center', enable_events=True), sg.Button('Applica modifica', key='-scaleConveyor')],
            [sg.Text('', size=(24, 2))],
            [sg.T('IMPOSTAZIONI GRAFICA', size=(30, 1), justification='left',font=('Any', 15, 'bold'))],
            [sg.Text('  Simulazione Grafica: ', size=(24, 1)), sg.Radio('Sì','UI', key='-gui', default=True, enable_events=True), sg.Radio('No', 'UI' ,key='-nogui', enable_events=True)],
            [sg.Text('', size=(24, 1))],
            ]    

#SCHEDA 'FISICA'
tab_fis = [[sg.Text('', size=(24, 1)),sg.Text('', size=(24, 1))],
            [sg.Text('Velocità conveyor:', size=(24, 1)), sg.InputText("0.8", key='-speed', size=(4, 1), justification='center', enable_events=True)],
            [sg.Text('Velocità troppo basse/alte possono causare glitch.', key = '-speedWarning' , visible=False, size=(50, 1))],
            [sg.Text('', size=(24, 1)),sg.Text('', size=(24, 1))],
            [sg.Text('% Bounciness degli oggetti: ', size=(24, 1)), sg.Slider(range=(0, 100),key='-bounObj', default_value=10, size=(15,10), orientation='horizontal')],
            [sg.Text('', size=(24, 1)),sg.Text('', size=(24, 1))],
            [sg.Text('% Bounciness del conveyor: ', size=(24, 1)), sg.Slider(range=(0, 100),key='-bounConv', default_value=10, size=(15,10),orientation='horizontal')],
            [sg.Text('', size=(24, 1)),sg.Text('', size=(24, 1))],
            [sg.Text('% Attrito degli oggetti:', size=(24, 1)),  sg.Slider(range=(0, 100), key='-attrObj',default_value=78, size=(15,10),orientation='horizontal')],
            [sg.Text('', size=(24, 1)),sg.Text('', size=(24, 1))],
            [sg.Text('% Attrito del conveyor:', size=(24, 1)), sg.Slider(range=(0, 100), key='-attrCon',default_value=78, size=(15,10),orientation='horizontal')],
            [sg.Text('', size=(24, 1)),sg.Text('', size=(24, 1))],
            [sg.Text('% Smorzamento lineare:', size=(24, 1)), sg.Slider(range=(0, 100), key='-smorLin',default_value=4, size=(15,10),orientation='horizontal')],
            [sg.Text('', size=(24, 1)),sg.Text('', size=(24, 1))],
            [sg.Text('% Smorzamento angolare:', size=(24, 1)), sg.Slider(range=(0, 100), key='-smorAng',default_value=4, size=(15,10),orientation='horizontal')],
            [sg.Text('', size=(24, 1)),sg.Text('', size=(24, 1))],
            [sg.Text('% Rolling friction:', size=(24, 1)), sg.Slider(range=(0, 100), key='-rolFrict',default_value=0, size=(15,10),orientation='horizontal')],
            [sg.Text('', size=(24, 1)),sg.Text('', size=(24, 1))],
            [sg.Text('% Spinning friction::', size=(24, 1)), sg.Slider(range=(0, 100), key='-spinFrict',default_value=0, size=(15,10),orientation='horizontal')],
            [sg.Text('', size=(24, 1)),sg.Text('', size=(24, 1))],
            [sg.Text('Massa degli oggetti: ', size=(24, 1)), sg.InputText("5", key='-weight',size=(4, 1), justification='center')],
            [sg.Text('', size=(24, 1)),sg.Text('', size=(24, 1))],
            ]    

#SCHEDA 'LOG'
tab_log = [[sg.Text('', size=(24, 1))],
            [sg.Text('Salvare dati in log? ', size=(24, 1)), sg.Radio('Sì','LOG', key='-log', default=True), sg.Radio('No', 'LOG' ,key='nolog')],
            [sg.Text('', size=(24, 1))],
            [sg.Text('Nome file per Log:', size=(24, 1)), sg.InputText(str(datetime.now())[0:-7], key='-logName', size=(20, 1), justification='right'), sg.Text('.csv', size=(5, 1))],
            [sg.Text('', size=(24, 1))],
            [sg.Text('Frequenza di estrazione dati: (sec)', size=(30, 1)), sg.InputText("1", key='-logFreq', size=(5, 1), justification='center')],
            [sg.Text('', size=(24, 1))],
            [sg.Text('Selezionare quali dati salvare nel log: ', size=(40, 1))],        
            [sg.Checkbox("#Oggetto", key='-logNum', default=True)],
            [sg.Checkbox("Tempo simulazione", key='-logTimeSim', default=True)],
            [sg.Checkbox("Posizione", key='-logPos', default=True)],
            [sg.Checkbox("Orientamento", key='-logOri', default=True)],
            [sg.Checkbox("Velocità lineare", key='-logVelLin', default=True)],
            [sg.Checkbox("Velocità angolare", key='-logVelAng', default=True)],
            [sg.Checkbox("Angolo iniziale", key='-logAng', default=True)]]

#SCHEDA PER IMPORTARE MODELLO
tab_mod = [[sg.Text('', size=(24, 1)),sg.Text('', size=(24, 1))],
            [sg.T('SCELTA OGGETTO', size=(30, 1), justification='left',font=('Any', 15,'bold'))],
            [sg.Text('Modello da simulare: ', size=(24, 1)), sg.InputText("wheel.urdf", key='-model',size=(25, 1), justification='center')],
            [sg.Text('Scala oggetto: ', size=(24, 1)), sg.InputText("1",key='-scale2', size=(4, 1), justification='center', enable_events=True), sg.Button('Anteprima', key='-preview2')],
            [sg.Text('', size=(24, 1)),sg.Text('', size=(24, 1))],
            [sg.Text('Modelli disponibili per la simulazione', size=(30,1))],
            [sg.Listbox(values=availableModels(), enable_events=True, size=(30,10),key='-fileList'), sg.Button('Aggiorna lista', size=(15, 10),key='-stlRefresh')],
             [sg.Text('', size=(24, 3))],

            [sg.T('CONVERTI STL IN URDF', size=(30, 1), justification='left',font=('Any', 15,'bold'))]
            ,[sg.Text('File STL da importare', size=(22, 1)), sg.Input('models/bunny.stl',key='-stlPath', enable_events=True,  size=(20,1), justification='right'), sg.FileBrowse("Esplora")],
           
            [sg.Text('Rinomina file da importare: ', size=(22, 1)), sg.InputText("nomeOggetto", key='-URDFname', size=(20,1), justification='right'), sg.Text('.urdf')],
            [sg.Text('', size=(15,1))],
            [sg.Text('',size=(18,1)), sg.Button('Importa modello', key='-createURDF')],
            [sg.Text('', size=(8,1)), sg.Text('', key='-importSuccess')],
            ]

#LAYOUT DELLE SCHEDE
layout = [[sg.TabGroup([[sg.Tab(' Impostazioni ', tab_sim), 
            sg.Tab(' Opzioni Fisica ', tab_fis), sg.Tab(' Opzioni Log ', tab_log), sg.Tab(' Importa modello ', tab_mod)]])],
            [sg.Button('AVVIA SIMULAZIONE', key='-Start', size=(23,3)), 
            sg.Button('INTERROMPI SIMULAZIONE', key='-Stop', visible=False, size=(30,3))] ]    


# Create the Window
window = sg.Window('CONVEYOR-SIMULATION', layout,alpha_channel=.99)
# Event Loop to process "ev
# ents" and get the "values" of the inputs
while True:
    event, values = window.read()
    
    
    if event == sg.WIN_CLOSED or event == 'Cancel': # if user closes window or clicks cancel
        break
    try:
        if (event == '-randAngles'):
            if values['-randAngles']==True:
                window['-ax'].update("0") 
                window['-ay'].update("0") 
                window['-az'].update("0") 
                
                

        if (event == '-nogui'):
            window['-numObj'].update("10",range=(1,1500))
            window['-limitNum'].update("[1-1500]") 
            
            
        if (event == '-gui'):
            window['-numObj'].update("2",range=(1,15))
            window['-limitNum'].update("[1-15]")
        
        if (event== '-createURDF'):
           createURDF(values['-stlPath'], values['-URDFname'])
           window['-importSuccess'].update(f"{values['-URDFname']} importato correttamente.")
           window['-fileList'].update(availableModels())
        
        if (event == '-stlRefresh'):
                window['-fileList'].update(availableModels())

        if (event == '-stlPath'):
            nameFile = values['-stlPath'].split("/")[len(values['-stlPath'].split("/"))-1].split(".")[0]
            window['-URDFname'].update(nameFile)
        
        if (event == '-fileList'):
            window['-model'].update(values['-fileList'][0])

        if (event == '-scaleConveyor'):
            modConveyor(values['-scaleConvX'], values['-scaleConvY'])
            
        try:
            if (event == '-speed'):
                if ((float(values['-speed']) < 0.3) | (float(values['-speed']) > 10)):
                    window['-speedWarning'].update(visible=True)
                elif ((float(values['-speed']) > 0.3) | (float(values['-speed']) < 10)):
                    window['-speedWarning'].update(visible=False)
        except:
            pass
        
        #Cosa loggare? array
        whatLog = [values['-logNum'], values['-logTimeSim'], values['-logPos'], values['-logOri'], values['-logVelLin'], values['-logVelAng'], values['-logAng']]
        #Quando si preme il tasto 'Start', viene chiamata la funzione per la simulazione
        if event in ('-Start'):
            window['-Stop'].update(visible=True)
            start(float(values['-speed']),         #VELOCITA
                int(values['-numObj']),          #NUMERO OGGETTI
                float(values['-freq']),       #FREQUENZA CADUTA
                float(values['-x']),        #PARTENZA DELLA CADUTA X
                float(values['-y']),         #PARTENZA DELLA CADUTA Y
                float(values['-z']),        #PARTENZA DELLA CADUTA Z
                float(values['-ax']),       #ANGOLO DELLA CADUTA X
                float(values['-ay']),         #ANGOLO DELLA CADUTA Y
                float(values['-az']),        #ANGOLO DELLA CADUTA Z
                values['-randAngles'],      #ANGOLI CASUALI BOOLEANO
                float(values['-bounObj'])/100,    #RIMBALZO OGGETII
                float(values['-bounConv'])/100,    #RIMBALZO CONVEYOR
                float(values['-attrObj'])/100,    #ATTRITO OGGETTI
                float(values['-attrCon'])/100,    #ATTRITO CONVEYOR
                float(values['-smorLin'])/100,    #SMORZAMENTO LINEARE
                float(values['-smorAng'])/100,      #SMORZAMENTO ANGOLARE
                float(values['-rolFrict'])/1000,    #ROLLING FRICTION
                float(values['-spinFrict'])/1000,    #SPINNING FRICTION
                float(values['-weight']),       #MASSA OGGETTI
                values['-gui'],            #ATTIVARE GRAFICA?
                values['-log'],              #SALVARE LOG?
                float(values['-logFreq']),             #FREQUENZA LOG 
                str(values['-model']),          #NOME MODELLO DA CARICARE
                float(values['-scale']),         #SCALA MODELLO
                str(values['-logName']),         #NOME FILE DI LOG
                whatLog,                            #ARRAY BOOL CON DATI DA SALVARE
                False  )
            
            window['-logName'].update(str(datetime.now())[0:-7])
                
        
        if event == '-scale':
            window['-scale2'].update(values['-scale'])
        if event == '-scale2':
            window['-scale'].update(values['-scale2'])
        
        #Preview quando si preme il tasto 'Anteprima'
        #Utile per posizionare l'oggetto nel punto desiderato e per verificarne le dimensioni
        if event in ('-preview', '-preview2'):
            start(0,         #VELOCITA
                1,          #NUMERO OGGETTI
                1,       #FREQUENZA CADUTA
                float(values['-x']),        #PARTENZA DELLA CADUTA X
                float(values['-y']),         #PARTENZA DELLA CADUTA Y
                float(values['-z']),        #PARTENZA DELLA CADUTA Z
                float(values['-ax']),        #PARTENZA DELLA CADUTA X
                float(values['-ay']),         #PARTENZA DELLA CADUTA Y
                float(values['-az']), 
                values['-randAngles'],
                0,    #RIMBALZO OGGETII
                0,    #RIMBALZO CONVEYOR
                0,    #ATTRITO OGGETTI
                0,    #ATTRITO CONVEYOR
                0,    #RIMBALZO OGGETII
                0,    #RIMBALZO CONVEYOR
                0,    #ATTRITO OGGETTI
                0,    #ATTRITO CONVEYOR
                3,       #MASSA OGGETTI
                True,            #ATTIVARE GRAFICA?
                False,              #SALVARE LOG?
                1000000,   
                str(values['-model']),          #NOME FILE
                float(values['-scale']),        #SCALA MODELLO
                "preview",
                [False, False, False, False, False, False, False], 
                True  )         
    except:
        raise Exception
        
        print("Invalid Values")

print(values)