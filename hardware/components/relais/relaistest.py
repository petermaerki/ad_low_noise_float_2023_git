import machine
import time

gpios_reset = [machine.Pin(i, machine.Pin.IN) for i in [10,11,12,13]]
gpios_set = [machine.Pin(i, machine.Pin.IN) for i in [18,19,20,21]]
adc = machine.ADC('GPIO26')

time.sleep_us(1000)

def schalten(set, puls_us):
    for pin in gpios_set:
        pin.value(set)
        pin.init(mode=machine.Pin.OUT)
    for pin in gpios_reset:
        pin.value(not set)
        pin.init(mode=machine.Pin.OUT)
    time.sleep_us(puls_us)
    for pin in gpios_set + gpios_reset:
        pin.init(mode=machine.Pin.IN)
        pin.value(0)
    time.sleep_us(100)
    
        
eingang_not_set = machine.Pin("GPIO16", machine.Pin.IN, machine.Pin.PULL_UP)
eingang_set = machine.Pin("GPIO17", machine.Pin.IN, machine.Pin.PULL_UP)


if True:
    R211_ohm = 1000
    R232_ohm = 1000
    R233_ohm = 1000
    # info  spannung_9V_grenze_V = 9 
    spannung_AD_grenze_V = 7.5
    spannung_AD_grenze_V = 3.0
    supply_Pi_V = 3.3
    AD_grenze_V = R233_ohm / (R211_ohm + R232_ohm + R233_ohm) * spannung_AD_grenze_V
    AD_grenze = AD_grenze_V / supply_Pi_V * 2**16
    print(AD_grenze)
    


    while True:
        if adc.read_u16() > AD_grenze and eingang_set.value() == False:
            # Supply high enough but relais still off
            schalten(set = True, puls_us=3000)
            print('Relais eingeschaltet')
            # Todo: Jetzt kann restliche Software anlaufen
            time.sleep_ms(1000)
        if adc.read_u16() < AD_grenze and eingang_set.value() == True:
            # Supply to low but relais still on
            schalten(set = False, puls_us=3000)
            print('Relais ausgeschaltet')
            # Todo: Jetzt Software neu starten
            time.sleep_ms(1000)
        #print(f'adc.read_u16 {adc.read_u16()}, AD_grenze {AD_grenze}, eingang_set {eingang_set.value()}')
        #time.sleep(1)
        

if False:
    stellung = True
    puls_us=3000
    while True:
        schalten(set= stellung, puls_us=puls_us)
        time.sleep_ms(100)
        ok= (stellung == eingang_set.value()) and (eingang_set.value() != eingang_not_set.value())
        print(f'{'korrekt geschaltet' if ok else 'fehler !!!!!!!!!!!'}, puls_us {puls_us}, stellung {stellung}, eingang_set {eingang_set.value()}, eingang_not_set {eingang_not_set.value()}')
        stellung = not stellung

if False:
    # Durationtest
    for puls_us in range(5000, 100, -100):
        for i in range(2):
            stellung = i%2
            schalten(set= stellung, puls_us=puls_us)
            time.sleep_ms(50)
            ok= (stellung == eingang_set.value()) and (eingang_set.value() != eingang_not_set.value())
            print(f'{'korrekt geschaltet' if ok else 'fehler !!!!!!!!!!!'}, puls_us {puls_us}, stellung {stellung}, eingang_set {eingang_set.value()}, eingang_not_set {eingang_not_set.value()}')
 
