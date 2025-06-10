/**
 * @file main.cpp
 * @brief Sistema de control ambiental y seguridad basado en Arduino.
 * @details Este proyecto implementa una máquina de estados para gestionar un sistema de seguridad
 * y control de confort ambiental. Utiliza un teclado para la autenticación, un lector RFID MFRC522
 * para leer un índice de confort (PMV), un sensor DHT11 para temperatura y humedad, y varios
 * actuadores como un ventilador, calefactor, LEDs y un buzzer para proporcionar feedback y
 * controlar el ambiente. El sistema opera en diferentes modos: seguridad, monitoreo,
 * enfriamiento, calefacción, alarma y bloqueo.
 * @author Duvan Alexix Hoyos Meneses, Royman Erira Benavidas, Sebastián Vallejo Palechor, Santiago Martínez Grajales
 * @date 11 de junio de 2024
 */
#include <LiquidCrystal.h>
#include <Keypad.h>
#include <DHT.h>
#include <SPI.h>
#include <MFRC522.h>
#include <StateMachineLib.h>
#include <AsyncTaskLib.h>
// ---------------------
// CONFIGURACIÓN DE HARDWARE
// ---------------------

/** @brief Pines de conexión para la pantalla LCD 16x2. */
const int LCD_RS = 12, LCD_EN = 11, LCD_D4 = 5, LCD_D5 = 4, LCD_D6 = 3, LCD_D7 = 2;
/** @brief Instancia del objeto para la pantalla LCD. */
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

/** @brief Número de filas del teclado matricial. */
const byte FILAS_TECLADO = 4;
/** @brief Número de columnas del teclado matricial. */
const byte COLUMNAS_TECLADO = 4;
/** @brief Pines de Arduino conectados a las filas del teclado. */
byte pinesFilas[FILAS_TECLADO] = {28, 30, 32, 34};
/** @brief Pines de Arduino conectados a las columnas del teclado. */
byte pinesColumnas[COLUMNAS_TECLADO] = {36, 38, 40, 42};
/** @brief Mapa de caracteres del teclado matricial 4x4. */
char teclas[FILAS_TECLADO][COLUMNAS_TECLADO] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};
/** @brief Instancia del objeto para el teclado. */
Keypad teclado = Keypad(makeKeymap(teclas), pinesFilas, pinesColumnas, FILAS_TECLADO, COLUMNAS_TECLADO);

/** @brief Pin SS (Slave Select) para el módulo RFID MFRC522. */
#define PIN_RFID_SS  45
/** @brief Pin de Reset para el módulo RFID MFRC522. */
#define PIN_RFID_RST 48
/** @brief Instancia del objeto para el lector RFID. */
MFRC522 lectorRFID(PIN_RFID_SS, PIN_RFID_RST);

/** @brief Pin de datos para el sensor de temperatura y humedad DHT11. */
#define PIN_DHT 14
/** @brief Tipo de sensor DHT utilizado (DHT11). */
#define TIPO_DHT DHT11
/** @brief Instancia del objeto para el sensor DHT. */
DHT sensorDHT(PIN_DHT, TIPO_DHT);

/** @brief Pin de control para el ventilador. */
#define PIN_VENTILADOR   35
/** @brief Pin de control para el calefactor. */
#define PIN_CALEFACTOR   8
/** @brief Pin de control para el buzzer. */
#define PIN_BUZZER       47
/** @brief Pin de control para el LED Rojo (indicador de alarma/bloqueo). */
#define PIN_LED_ROJO     25
/** @brief Pin de control para el LED Verde (indicador de éxito/calefacción). */
#define PIN_LED_VERDE    37
/** @brief Pin de control para el LED Azul (indicador de error/enfriamiento). */
#define PIN_LED_AZUL     49
/** @brief Pin analógico para el sensor de luz (LDR). */
#define PIN_SENSOR_LUZ   A0

// ---------------------
// DEFINICIÓN DE ESTADOS
// ---------------------

/**
 * @enum EstadosSistema
 * @brief Define los posibles estados de operación de la máquina de estados.
 */
// ---------------------
// DEFINICIÓN DE ESTADOS
// ---------------------
enum EstadosSistema {
    MODO_SEGURIDAD = 0,    ///< Estado inicial, esperando autenticación por clave.
    MODO_MONITOREO,        ///< Monitorea las condiciones ambientales y espera cambios.
    MODO_ENFRIAMIENTO,     ///< Activa el ventilador si el PMV es alto.
    MODO_CALEFACCION,      ///< Activa el calefactor si el PMV es bajo.
    MODO_ALARMA,           ///< Se activa bajo condiciones anómalas (ej. alta temp y baja luz).
    MODO_BLOQUEO           ///< El sistema se bloquea tras múltiples fallos o alarmas.
};
/** @brief Instancia de la máquina de estados. */
StateMachine maquinaEstados(6, 10);
/** @brief Variable que almacena el estado actual del sistema. */
EstadosSistema estadoActual = MODO_SEGURIDAD;

// ---------------------
// VARIABLES GLOBALES
// ---------------------

/** @brief Almacena la clave introducida por el usuario en el teclado. */
String entradaClave = "";
/** @brief Clave correcta para la autenticación. */
const String CLAVE_CORRECTA = "1234";
/** @brief Contador de intentos de clave fallidos. */
int intentosFallidos = 0;
/** @brief Bandera que indica si el usuario se ha autenticado correctamente. */
bool autenticado = false;
/** @brief Bandera para solicitar el desbloqueo desde el modo de bloqueo. */
bool desbloqueoActivado = false;
/** @brief Contador de alarmas consecutivas para determinar si se debe bloquear el sistema. */
int alarmasConsecutivas = 0;

/** @brief Almacena la última lectura de temperatura. */
float temperatura = 0.0;
/** @brief Almacena la última lectura de humedad. */
float humedad = 0.0;
/** @brief Almacena la última lectura del sensor de luz. */
int nivelLuz = 0;
/** @brief Voto Medio Previsto (Predicted Mean Vote), leído desde la tarjeta RFID. */
float indicePMV = 0.0;

/** @brief Almacena el tiempo (en ms) en que se entró al estado actual. */
unsigned long tiempoInicioEstado = 0;
/** @brief Almacena el tiempo (en ms) en que se mostró un mensaje de error. */
unsigned long tiempoInicioError = 0;
/** @brief Bandera para controlar la visualización de mensajes de error. */
bool mostrandoError = false;

/** @brief Temporizador para el parpadeo de la alarma. */
unsigned long tiempoAnteriorAlarma = 0;
/** @brief Temporizador para el parpadeo del LED en modo bloqueo. */
unsigned long tiempoAnteriorBloqueo = 0;
/** @brief Temporizador para el ciclo de encendido/apagado del ventilador. */
unsigned long tiempoUltimoCambioVentilador = 0;
/** @brief Intervalo de tiempo (en ms) para el ciclo del ventilador en modo normal. */
const unsigned long INTERVALO_VENTILADOR = 20000;
/** @brief Bandera para forzar la ventilación continua si el PMV es muy alto. */
bool ventilacionForzada = false;

// ---------------------
// FUNCIONES RFID
// ---------------------

/**
 * @brief Lee el valor de PMV (float) desde el bloque 4 de una tarjeta MIFARE.
 * @param[out] valorPMV Referencia a un float donde se almacenará el valor leído.
 * @return `true` si la lectura fue exitosa, `false` en caso contrario.
 */
bool leerPMVdeTarjeta(float &valorPMV) {
    if (!lectorRFID.PICC_IsNewCardPresent() || !lectorRFID.PICC_ReadCardSerial()) {
        return false;
    }

    byte blockAddr = 4;
    MFRC522::MIFARE_Key key;
    for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF; // Clave por defecto

    byte buffer[18];
    byte size = sizeof(buffer);

    MFRC522::StatusCode status = lectorRFID.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, blockAddr, &key, &(lectorRFID.uid));
    if (status != MFRC522::STATUS_OK) {
        lectorRFID.PCD_StopCrypto1();
        return false;
    }

    status = lectorRFID.MIFARE_Read(blockAddr, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        lectorRFID.PCD_StopCrypto1();
        return false;
    }

    // Convertir los primeros 4 bytes del buffer a un float
    memcpy(&valorPMV, buffer, sizeof(float));

    lectorRFID.PICC_HaltA();
    lectorRFID.PCD_StopCrypto1();
    return true;
}


/**
 * @brief Graba un valor de PMV (float) en el bloque 4 de una tarjeta MIFARE.
 * @param[in] valorPMV El valor float que se desea grabar.
 * @param[in] detenerComunicacion Si es true, detiene la comunicación después de escribir.
 * @return `true` si la escritura fue exitosa, `false` en caso contrario.
 */
bool grabarPMVenTarjeta(float valorPMV, bool detenerComunicacion = true) {
    if (!lectorRFID.PICC_IsNewCardPresent() || !lectorRFID.PICC_ReadCardSerial()) {
        return false;
    }
    
    // UID de la tarjeta para referencia
    Serial.print("UID de la tarjeta: ");
    for (byte i = 0; i < lectorRFID.uid.size; i++) {
        Serial.print(lectorRFID.uid.uidByte[i] < 0x10 ? " 0" : " ");
        Serial.print(lectorRFID.uid.uidByte[i], HEX);
    }
    Serial.println();

    byte blockAddr = 4;
    MFRC522::MIFARE_Key key;
    for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF; // Clave por defecto

    MFRC522::StatusCode status = lectorRFID.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, blockAddr, &key, &(lectorRFID.uid));
    if (status != MFRC522::STATUS_OK) {
        if (detenerComunicacion) lectorRFID.PCD_StopCrypto1();
        return false;
    }

    byte buffer[16] = {0};
    memcpy(buffer, &valorPMV, sizeof(float));

    status = lectorRFID.MIFARE_Write(blockAddr, buffer, 16);
    if (status != MFRC522::STATUS_OK) {
        if (detenerComunicacion) lectorRFID.PCD_StopCrypto1();
        return false;
    }

    // Detener la comunicación con la tarjeta si se solicita
    if (detenerComunicacion) {
        lectorRFID.PICC_HaltA();
        lectorRFID.PCD_StopCrypto1();
    }
    digitalWrite(PIN_LED_VERDE, LOW);
    
    return true; // Simplificado, la verificación de re-lectura se omite por brevedad.
}
/**
 * @brief Graba diferentes valores PMV en diferentes tarjetas de forma secuencial.
 * @details Esta función guía al usuario para que acerque varias tarjetas RFID una por una
 * y graba en cada una un valor de PMV de un array proporcionado.
 * @param[in] valoresPMV Array de valores float a grabar.
 * @param[in] numTarjetas Número de valores/tarjetas a grabar.
 * @param[in] tiempoEspera Tiempo máximo en ms para esperar por cada tarjeta.
 * @return El número de tarjetas que fueron grabadas exitosamente.
 */
int grabarDiferentesPMVEnTarjetas(float valoresPMV[], int numTarjetas, unsigned long tiempoEspera = 10000) {
    int tarjetasGrabadas = 0;
    Serial.println("Iniciando grabacion secuencial de PMV.");
    Serial.println("Acerque las tarjetas una por una.");

    for (int i = 0; i < numTarjetas; i++) {
        digitalWrite(PIN_LED_VERDE, LOW);
        Serial.print("Acerque tarjeta ");
        Serial.print(i + 1);
        Serial.print(" para grabar PMV: ");
        Serial.println(valoresPMV[i], 2);

        unsigned long tiempoInicio = millis();
        bool grabadoExitoso = false;

        while (millis() - tiempoInicio < tiempoEspera && !grabadoExitoso) {
            if (grabarPMVenTarjeta(valoresPMV[i], true)) {
                tarjetasGrabadas++;
                grabadoExitoso = true;
                Serial.println("Tarjeta grabada exitosamente.");
                
                digitalWrite(PIN_LED_VERDE, HIGH);
                tone(PIN_BUZZER, 1000, 200);
                delay(300); // Feedback
                digitalWrite(PIN_LED_VERDE, LOW);
            }
            delay(100);
        }

        if (grabadoExitoso) {
            if (i < numTarjetas - 1) {
                Serial.println("Retire la tarjeta y prepare la siguiente.");
                delay(1000);
            }
        } else {
            Serial.println("Tiempo de espera agotado.");
            break;
        }
    }
    digitalWrite(PIN_LED_VERDE, LOW);
    return tarjetasGrabadas;
}

// ---------------------
// TAREAS ASÍNCRONAS
// ---------------------
/** @brief Tarea asíncrona para leer los sensores DHT11 y de luz cada 2 segundos. */
AsyncTask tareaSensores(2000, true, []() {
    if (estadoActual != MODO_MONITOREO) return;
    float tempLeida = sensorDHT.readTemperature();
    float humLeida = sensorDHT.readHumidity();
    if (!isnan(tempLeida)) temperatura = tempLeida;
    if (!isnan(humLeida)) humedad = humLeida;
    nivelLuz = analogRead(PIN_SENSOR_LUZ);
});

/** @brief Tarea asíncrona para leer la tarjeta RFID cada segundo y actualizar el PMV. */
AsyncTask tareaRFID(1000, true, []() {
    if (estadoActual != MODO_MONITOREO && estadoActual != MODO_SEGURIDAD) return;
    float pmvLeido;
    if (leerPMVdeTarjeta(pmvLeido)) {
        if (abs(pmvLeido - indicePMV) > 0.1f) { // Actualizar solo si hay cambio
            indicePMV = pmvLeido;
            Serial.print("PMV actualizado desde tarjeta: ");
            Serial.println(indicePMV, 2);
            digitalWrite(PIN_LED_VERDE, HIGH);
            delay(100);
            digitalWrite(PIN_LED_VERDE, LOW);
            
            if (estadoActual == MODO_MONITOREO) { // Actualizar LCD
                lcd.setCursor(0, 1);
                lcd.print("L:" + String(nivelLuz) + " PMV:" + String(indicePMV, 1) + "  ");
            }
        }
    }
});

/** @brief Tarea asíncrona para actualizar la información en la pantalla LCD cada segundo. */
AsyncTask tareaLCD(1000, true, []() {
    if (estadoActual == MODO_MONITOREO) {
        lcd.setCursor(0, 0);
        lcd.print("T:" + String(temperatura, 1) + "C H:" + String(humedad, 0) + "%  ");
        lcd.setCursor(0, 1);
        lcd.print("L:" + String(nivelLuz) + " PMV:" + String(indicePMV, 1) + "  ");
    }
});

/** @brief Tarea asíncrona para procesar la entrada del teclado cada 100 ms. */
AsyncTask tareaTeclado(100, true, []() {
    char tecla = teclado.getKey();
    if (!tecla) return;

    if (estadoActual == MODO_SEGURIDAD && !mostrandoError) {
        if (tecla >= '0' && tecla <= '9' && entradaClave.length() < 4) {
            entradaClave += tecla;
            lcd.setCursor(entradaClave.length() - 1, 1);
            lcd.print('*');
            
            if (entradaClave.length() == 4) {
                if (entradaClave == CLAVE_CORRECTA) {
                    autenticado = true;
                } else {
                    intentosFallidos++;
                    digitalWrite(PIN_LED_AZUL, HIGH);
                    entradaClave = "";
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("Clave Incorrecta");
                    lcd.setCursor(0, 1);
                    lcd.print("Intento " + String(intentosFallidos) + "/3");
                    mostrandoError = true;
                    tiempoInicioError = millis();
                }
            }
        }
    } else if (estadoActual == MODO_BLOQUEO && tecla == '#') {
        desbloqueoActivado = true;
    }
});

/** @brief Tarea asíncrona para limpiar el buffer serial cada 10 segundos. */
AsyncTask tareaLimpiezaSerial(10000, true, []() {
    while (Serial.available() > 0) Serial.read();
});

/** @brief Tarea asíncrona para controlar el ciclo del ventilador cada 200 ms. */
AsyncTask tareaVentilador(200, true, []() {
    if (estadoActual != MODO_ENFRIAMIENTO && !ventilacionForzada) return;

    if (ventilacionForzada) { // Modo PMV alto
        if (digitalRead(PIN_VENTILADOR) != HIGH) {
            digitalWrite(PIN_VENTILADOR, HIGH);
            lcd.setCursor(0, 1);
            lcd.print("Ventilador ON   ");
        }
    } else { // Modo enfriamiento normal
        if (millis() - tiempoUltimoCambioVentilador >= INTERVALO_VENTILADOR) {
            digitalWrite(PIN_VENTILADOR, !digitalRead(PIN_VENTILADOR));
            tiempoUltimoCambioVentilador = millis();
            lcd.setCursor(0, 1);
            lcd.print(digitalRead(PIN_VENTILADOR) == HIGH ? "Ventilador ON   " : "Ventilador OFF  ");
        }
    }
});

// ---------------------
// FUNCIONES DE ESTADO
// ---------------------
/** @brief Acciones a realizar al salir del estado de alarma. Apaga el buzzer y el LED rojo. @see MODO_ALARMA */
void salidaModoAlarma() {
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_LED_ROJO, LOW);
}

/** @brief Acciones a realizar al salir del modo de bloqueo. Apaga el LED rojo. @see MODO_BLOQUEO */
void salidaModoBloqueo() {
    digitalWrite(PIN_LED_ROJO, LOW);
}

/** @brief Acciones a realizar al salir del modo de enfriamiento. Apaga el ventilador y el LED azul. @see MODO_ENFRIAMIENTO */
void salidaModoEnfriamiento() {
    digitalWrite(PIN_VENTILADOR, LOW);
    digitalWrite(PIN_LED_AZUL, LOW);
}

/** @brief Acciones a realizar al salir del modo de calefacción. Apaga el calefactor y el LED verde. @see MODO_CALEFACCION */
void salidaModoCalefactor() {
    digitalWrite(PIN_LED_VERDE, LOW);
    digitalWrite(PIN_CALEFACTOR, LOW);
}

/** @brief Acciones de inicialización al entrar en el modo de seguridad. @see MODO_SEGURIDAD */
void entradaModoSeguridad() {
    estadoActual = MODO_SEGURIDAD;
    entradaClave = "";
    intentosFallidos = 0;
    autenticado = false;
    desbloqueoActivado = false;
    alarmasConsecutivas = 0;

    digitalWrite(PIN_VENTILADOR, LOW);
    digitalWrite(PIN_CALEFACTOR, LOW);
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_LED_ROJO, LOW);
    digitalWrite(PIN_LED_VERDE, LOW);
    digitalWrite(PIN_LED_AZUL, LOW);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Ingrese clave:");
}

/** @brief Acciones de inicialización al entrar en el modo de monitoreo. @see MODO_MONITOREO */
void entradaModoMonitoreo() {
    estadoActual = MODO_MONITOREO;
    digitalWrite(PIN_VENTILADOR, LOW);
    digitalWrite(PIN_CALEFACTOR, LOW);
    tiempoInicioEstado = millis();
    lcd.clear();
    tareaLCD.Update();
}

/** @brief Acciones de inicialización al entrar en el modo de enfriamiento. @see MODO_ENFRIAMIENTO */
void entradaModoEnfriamiento() {
    estadoActual = MODO_ENFRIAMIENTO;
    alarmasConsecutivas = 0;
    digitalWrite(PIN_VENTILADOR, HIGH); // Encender ventilador al entrar
    tiempoInicioEstado = millis();
    tiempoUltimoCambioVentilador = millis();
    digitalWrite(PIN_LED_AZUL, HIGH);
    lcd.clear();
    lcd.print("PMV Alto");
    lcd.setCursor(0, 1);
    lcd.print("Ventilador ON ");
}

/** @brief Acciones de inicialización al entrar en el modo de calefacción. @see MODO_CALEFACCION */
void entradaModoCalefaccion() {
    estadoActual = MODO_CALEFACCION;
    alarmasConsecutivas = 0;
    digitalWrite(PIN_CALEFACTOR, HIGH);
    tiempoInicioEstado = millis();
    digitalWrite(PIN_LED_VERDE, HIGH);
    lcd.clear();
    lcd.print("PMV Bajo");
    lcd.setCursor(0, 1);
    lcd.print("Calefactor ON");
}

/** @brief Acciones de inicialización al entrar en el modo de alarma. @see MODO_ALARMA */
void entradaModoAlarma() {
    estadoActual = MODO_ALARMA;
    tiempoInicioEstado = millis();
    alarmasConsecutivas++;
    tiempoAnteriorAlarma = millis();
    digitalWrite(PIN_LED_ROJO, HIGH);
    digitalWrite(PIN_BUZZER, HIGH);
    lcd.clear();
    lcd.print("!! ALARMA !!");
    lcd.setCursor(0, 1);
    lcd.print("Condic. Anormal");
}

/** @brief Acciones de inicialización al entrar en el modo de bloqueo. @see MODO_BLOQUEO */
void entradaModoBloqueo() {
    estadoActual = MODO_BLOQUEO;
    digitalWrite(PIN_VENTILADOR, LOW);
    digitalWrite(PIN_CALEFACTOR, LOW);
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_LED_ROJO, HIGH);
    tiempoAnteriorBloqueo = millis();
    lcd.clear();
    lcd.print("SIST. BLOQUEADO");
    lcd.setCursor(0, 1);
    lcd.print("Presione #");
}

// ---------------------
// FUNCIONES AUXILIARES
// ---------------------
/** @brief Gestiona la visualización temporal de mensajes de error en la LCD. */
void manejarMensajeError() {
    if (mostrandoError && (millis() - tiempoInicioError >= 2000)) {
        mostrandoError = false;
        digitalWrite(PIN_LED_AZUL, LOW);
        if (intentosFallidos < 3) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Ingrese clave:");
        }
    }
}

/** @brief Controla el parpadeo del LED rojo y el sonido del buzzer en el modo de alarma. */
void parpadeoAlarma() {
    if (estadoActual != MODO_ALARMA) return;
    unsigned long tiempoActual = millis();
    if (digitalRead(PIN_LED_ROJO) == HIGH && tiempoActual - tiempoAnteriorAlarma >= 200) {
        digitalWrite(PIN_LED_ROJO, LOW);
        digitalWrite(PIN_BUZZER, LOW);
        tiempoAnteriorAlarma = tiempoActual;
    } else if (digitalRead(PIN_LED_ROJO) == LOW && tiempoActual - tiempoAnteriorAlarma >= 100) {
        digitalWrite(PIN_LED_ROJO, HIGH);
        digitalWrite(PIN_BUZZER, HIGH);
        tiempoAnteriorAlarma = tiempoActual;
    }
}

/** @brief Controla el parpadeo del LED rojo en el modo de bloqueo del sistema. */
void parpadeoBloqueo() {
    if (estadoActual != MODO_BLOQUEO) return;
    unsigned long tiempoActual = millis();
    if (digitalRead(PIN_LED_ROJO) == HIGH && tiempoActual - tiempoAnteriorBloqueo >= 500) {
        digitalWrite(PIN_LED_ROJO, LOW);
        tiempoAnteriorBloqueo = tiempoActual;
    } else if (digitalRead(PIN_LED_ROJO) == LOW && tiempoActual - tiempoAnteriorBloqueo >= 200) {
        digitalWrite(PIN_LED_ROJO, HIGH);
        tiempoAnteriorBloqueo = tiempoActual;
    }
}

// ---------------------
// CONFIGURACIÓN DE MÁQUINA DE ESTADOS
// ---------------------
/**
 * @brief Define todas las transiciones y las funciones de entrada/salida para la máquina de estados.
 * @details Configura las condiciones bajo las cuales el sistema cambia de un estado a otro
 * y asocia las funciones que deben ejecutarse al entrar o salir de cada estado.
 */
void configurarMaquinaEstados() {
    maquinaEstados.AddTransition(MODO_SEGURIDAD, MODO_MONITOREO, []() { return autenticado; });
    maquinaEstados.AddTransition(MODO_SEGURIDAD, MODO_BLOQUEO, []() { return intentosFallidos >= 3; });
    maquinaEstados.AddTransition(MODO_MONITOREO, MODO_ALARMA, []() { return (millis() - tiempoInicioEstado >= 1500) && (temperatura > 40 && nivelLuz < 10); });
    maquinaEstados.AddTransition(MODO_MONITOREO, MODO_ENFRIAMIENTO, []() { return (millis() - tiempoInicioEstado >= 1500) && (indicePMV > 1); });
    maquinaEstados.AddTransition(MODO_MONITOREO, MODO_CALEFACCION, []() { return (millis() - tiempoInicioEstado >= 1500) && (indicePMV < -1); });
    maquinaEstados.AddTransition(MODO_ENFRIAMIENTO, MODO_MONITOREO, []() { return (millis() - tiempoInicioEstado >= 7000); });
    maquinaEstados.AddTransition(MODO_CALEFACCION, MODO_MONITOREO, []() { return (millis() - tiempoInicioEstado >= 4000); });
    maquinaEstados.AddTransition(MODO_ALARMA, MODO_BLOQUEO, []() { return alarmasConsecutivas >= 3; });
    maquinaEstados.AddTransition(MODO_ALARMA, MODO_MONITOREO, []() { return (millis() - tiempoInicioEstado >= 5000) && (alarmasConsecutivas < 3); });
    maquinaEstados.AddTransition(MODO_BLOQUEO, MODO_SEGURIDAD, []() { return desbloqueoActivado; });

    maquinaEstados.SetOnEntering(MODO_SEGURIDAD, entradaModoSeguridad);
    maquinaEstados.SetOnEntering(MODO_MONITOREO, entradaModoMonitoreo);
    maquinaEstados.SetOnEntering(MODO_ENFRIAMIENTO, entradaModoEnfriamiento);
    maquinaEstados.SetOnEntering(MODO_CALEFACCION, entradaModoCalefaccion);
    maquinaEstados.SetOnEntering(MODO_ALARMA, entradaModoAlarma);
    maquinaEstados.SetOnEntering(MODO_BLOQUEO, entradaModoBloqueo);
    
    maquinaEstados.SetOnLeaving(MODO_ALARMA, salidaModoAlarma);
    maquinaEstados.SetOnLeaving(MODO_BLOQUEO, salidaModoBloqueo);
    maquinaEstados.SetOnLeaving(MODO_ENFRIAMIENTO, salidaModoEnfriamiento);
    maquinaEstados.SetOnLeaving(MODO_CALEFACCION, salidaModoCalefactor);
}

// ---------------------
// SETUP
// ---------------------
/**
 * @brief Función de configuración principal. Se ejecuta una vez al iniciar el microcontrolador.
 * @details Inicializa la comunicación serial, los pines de I/O, la LCD, los sensores,
 * el lector RFID, configura la máquina de estados e inicia las tareas asíncronas.
 * También ejecuta una rutina para grabar valores iniciales de PMV en dos tarjetas.
 */
void setup() {
    Serial.begin(9600);
    
    pinMode(PIN_VENTILADOR, OUTPUT);
    pinMode(PIN_CALEFACTOR, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LED_ROJO, OUTPUT);
    pinMode(PIN_LED_VERDE, OUTPUT);
    pinMode(PIN_LED_AZUL, OUTPUT);
    
    lcd.begin(16, 2);
    lcd.print("Iniciando...");
    delay(500);
    
    sensorDHT.begin();
    SPI.begin();
    lectorRFID.PCD_Init();
    
    configurarMaquinaEstados();
    maquinaEstados.SetState(MODO_SEGURIDAD, false, true);

    tareaSensores.Start();
    tareaRFID.Start();
    tareaLCD.Start();
    tareaTeclado.Start();
    tareaLimpiezaSerial.Start();
    tareaVentilador.Start();
    
    // Graba un set inicial de tarjetas al iniciar
    float valoresPMV[] = {-1.5, 1.5}; // Valores para calefacción y ventilación
    int numTarjetas = 2;
    grabarDiferentesPMVEnTarjetas(valoresPMV, numTarjetas);
}

// ---------------------
// LOOP
// ---------------------
/**
 * @brief Bucle principal del programa. Se ejecuta continuamente después del setup().
 * @details En cada ciclo, actualiza el estado de todas las tareas asíncronas,
 * gestiona los parpadeos de los LEDs de estado, maneja los mensajes de error y
 * actualiza la máquina de estados para verificar si debe ocurrir una transición.
 */
void loop() {
    tareaSensores.Update();
    tareaRFID.Update();
    tareaLCD.Update();
    tareaTeclado.Update();
    tareaLimpiezaSerial.Update();
    tareaVentilador.Update();
    
    parpadeoAlarma();
    parpadeoBloqueo();
    
    manejarMensajeError();
    
    maquinaEstados.Update();
}
