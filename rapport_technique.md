# Rapport Technique - Système de Pesage Électronique

## Table des Matières
1. [Vue d'ensemble](#vue-densemble)
2. [Architecture Logicielle](#architecture-logicielle)
3. [Gestion des Entrées/Sorties](#gestion-des-entreéssosties)
4. [Calculs de Pesage](#calculs-de-pesage)
5. [Tests Automatiques](#tests-automatiques)
6. [Justificatifs de Conception](#justificatifs-de-conception)

---

## Vue d'ensemble

Le système de pesage électronique est développé sur une **plateforme ESP32** avec une architecture modulaire basée sur **FreeRTOS** et le framework **ESP-IDF**. Le système intègre plusieurs composants matériels (capteur de pression, boutons, LEDs, buzzer, écran LCD) et met en place une architecture logicielle décentralisée basée sur la communication par queues de messages.

**Cible matérielle :** ESP32
**Système d'exploitation temps réel :** FreeRTOS
**Framework :** ESP-IDF (Espressif IoT Development Framework)

---

## Architecture Logicielle

### 1.1 Principes Architecturaux

L'architecture repose sur trois principes fondamentaux :

#### **1. Modularité par Composants**
Chaque périphérique est encapsulé dans un composant indépendant avec une interface claire :
- `pressure/` : gestion du capteur de poids
- `button/` : gestion des boutons tactiles
- `led/` : gestion des indicateurs lumineux
- `buzzer/` : gestion du signal sonore
- `lcd/` : gestion de l'affichage
- `autotest/` : tests automatiques des composants

#### **2. Communication par Queues Asynchrone**
Les composants communiquent via des **queues FreeRTOS** plutôt que par appels directs. Cela offre :
- Découplage entre producteurs et consommateurs
- Non-blocage des tâches
- Scalabilité facile

#### **3. Architecture Multi-Tâches**
Chaque composant d'entrée/sortie s'exécute dans sa propre tâche FreeRTOS :
```
main() 
  ├─ pressure_task     (acquisition capteur)
  ├─ button_task       (polling bouton 1)
  ├─ button_task       (polling bouton 2)
  └─ lcd_task          (affichage résultats)
```

### 1.2 Flux de Données Logique

```
CAPTEURS (Entrées)
    ↓
[msg_q_sensor] ← pressure_task, button_task, button_task
    ↓
app_main (boucle principale)
    │
    ├─→ Traitement événements
    │   ├─ Activation/désactivation veille
    │   ├─ Tarage capteur
    │   └─ Contrôle LEDs/Buzzer
    │
    └─→ [msg_q_lcd]
         ↓
      lcd_task (affichage)
         ↓
    AFFICHEUR LCD (Sortie)
```

### 1.3 Structures de Données Centrales

#### **Message Universel (`message.h`)**
```c
typedef struct msg_t {
    uint8_t id;        // Identifiant du capteur/composant
    uint32_t value;    // Valeur transmise
} msg_t;
```

Cette structure minimale est utilisée par tous les composants pour garantir une communication uniforme.

#### **Structures par Composant**

**Pression (Capteur de Poids):**
```c
typedef struct pressure_t {
    uint8_t id;                  // ID unique
    gpio_num_t pin;              // PIN ADC
    uint32_t reset_val;          // Valeur de réinitialisation
    _lock_t mutex;               // Protection d'accès concurrent
    QueueHandle_t msg_q_sensor;  // Queue d'envoi des messages
} pressure_t;
```

**Bouton:**
```c
typedef struct button_t {
    uint8_t id;
    uint8_t is_pressed;          // État actuel
    gpio_num_t pin;
    _lock_t mutex;
    QueueHandle_t msg_q_sensor;
} button_t;
```

**Autotest:**
```c
typedef struct autotest_t {
    led_t *led1, *led2;
    buzzer_t *buzzer;
    button_t *button1, *button2;
    lcd_t *lcd;
    pressure_t *pressure;
} autotest_t;  // Agrégation de tous les composants
```

---

## Gestion des Entrées/Sorties

### 2.1 Entrées : Capteur de Pression (ADC)

#### **Configuration Matérielle**
- **Canal ADC :** ADC2_CHANNEL_2 (GPIO2)
- **Résolution :** 12 bits (valeurs 0-4095)
- **Atténuation :** ADC_ATTEN_DB_11 (atténuation maximale pour plage 0-3.9V)
- **Calibrage :** Utilisation de la table de calibrage legacy ESP-IDF

#### **Pipeline d'Acquisition**
```
pressure_task() exécutée toutes les 100ms
    ├─ pressure_read_raw()      → ADC brut (0-4095)
    ├─ pressure_read_voltage()  → Conversion en volts
    └─ pressure_read_weight()   → Conversion en grammes
         ↓
    Moyennes mobiles (10 dernières lectures)
         ↓
    Envoi sur msg_q_sensor tous les 1000ms (après 10 lectures)
```

#### **Moyennes Mobiles**
Un buffer circulaire de 10 valeurs pondère les lectures pour améliorer la stabilité :
```c
static int32_t values[10] = {0};
static size_t values_index = 0;

// Mise à jour circulaire
values[values_index] = (int32_t)roundf(weight);
values_index = (values_index + 1) % 10;

// Envoi de la moyenne tous les 10 cycles (1 seconde)
msg.value = mean(values, 10);
```

**Avantage :** Lissage du bruit ADC sans latence excessive.

### 2.2 Entrées : Boutons Tactiles

#### **Configuration**
- **Pins :** GPIO_NUM_33 (Bouton 1), GPIO_NUM_32 (Bouton 2)
- **Mode :** GPIO_PULLUP_ONLY (pull-up interne)
- **Logique :** Actif bas (appui = 0, repos = 1)

#### **Dédoublement du Contact**
```c
void button_update_state(button_t *button) {
    // Première lecture
    uint8_t read1 = gpio_get_level(button->pin);
    vTaskDelay(50 / portTICK_PERIOD_MS);  // Attendre 50ms
    // Deuxième lecture
    uint8_t read2 = gpio_get_level(button->pin);
    
    // Validé que si les deux lectures concordent
    if (read1 == read2) {
        button->is_pressed = (read1 == 0) ? 1 : 0;
    }
}
```

**Justification :** Élimine les rebonds électromagnétiques sur 50ms.

#### **Détection d'Évènement**
La tâche compare l'état courant avec l'état précédent et envoie un message **uniquement lors d'un changement** :
```c
if (button->is_pressed != last_state) {
    xQueueSend(button->msg_q_sensor, &msg, portMAX_DELAY);
    last_state = button->is_pressed;
}
vTaskDelay(100 / portTICK_PERIOD_MS);
```

**Avantage :** Réduit le bruit de la file de messages.

### 2.3 Sorties : LEDs

Configuration simple via GPIO en mode output :
```c
void led_init(led_t *led, gpio_num_t pin, uint8_t id) {
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    led->pin = pin;
    led->id = id;
}
```

**Commandes :**
- `led_on()` / `led_off()` : Allumage/extinction
- `led_toggle()` : Basculement
- `led_blink()` : Clignotement avec période configurable

### 2.4 Sorties : Buzzer

Similaire aux LEDs, utilise le même modèle GPIO pour générer un signal sonore.

### 2.5 Sorties : Écran LCD 16×2

#### **Interface I2C**
- **SCL :** GPIO27
- **SDA :** GPIO14
- **Adresse :** 0x3E (module LCD rétro-éclairé via I2C)

#### **Étapes 2→4 : Conversion, Modèle de Jauge et Poids**

Le code a été modifié pour effectuer la conversion ADC→tension puis estimer la résistance de la jauge et en déduire la force par un modèle de type "power-law" sur la résistance. La chaîne actuelle implémente les fonctions suivantes :

- `convert_adc_to_voltage(adc_raw)` : conversion linéaire simple de la valeur brute ADC vers une tension (utilise `V_REF` et `ADC_MAX_VALUE`).
- `calculate_r_jauge(V_mesuree)` : calcul de la résistance de la jauge à partir de la tension mesurée en inverse du pont diviseur :
  $$R_{jauge} = \frac{V_{mesuree} \cdot R_M}{V_{supply} - V_{mesuree}}$$
- `calculate_force_from_r_jauge(R_jauge)` : inversion du modèle de jauge utilisé en calibration :
  Si le modèle est donné par $R_{jauge} = a \cdot Force^{-b}$ alors
  $$Force = \left(\frac{a}{R_{jauge}}\right)^{1/b}$$

La fonction de haut niveau `get_final_weight_g()` :

1. Moyenne des 10 dernières lectures brutes ADC.
2. Conversion ADC → tension via `convert_adc_to_voltage()`.
3. Calcul de `R_jauge` via `calculate_r_jauge()`.
4. Calcul du poids brut via `calculate_force_from_r_jauge()`.
5. Application d'un facteur et soustraction de l'offset de tarage (`CALIBRATION_OFFSET_G`).

Extrait simplifié :

```c
float V_mesuree = convert_adc_to_voltage(adc_average);
float R_jauge  = calculate_r_jauge(V_mesuree);
float weight_brut = calculate_force_from_r_jauge(R_jauge);
float final_weight = weight_brut * 100 - CALIBRATION_OFFSET_G;
if (final_weight < 0.0) final_weight = 0.0;
```

Remarques importantes :
- Le modèle logarithmique décrit précédemment a été commenté dans le code et remplacé par le modèle basé sur la relation $R_{jauge} = a \cdot Force^{-b}$ (coefficients `CALIBRATION_COEF_A` et `CALIBRATION_COEF_B`).
- Les valeurs de `V_SUPPLY` (4.8V), `R_M` (10 kOhm), `V_REF` (5V) et le multiplicateur `*100` appliqué au résultat proviennent de l'implémentation et doivent être validés lors de la calibration.
- L'offset de tarage est stocké dans `CALIBRATION_OFFSET_G` (mémorisé en NVM dans l'application).
```


### 3.2 Justification du Modèle

Les jauges de contrainte peuvent être modélisées par une relation non-linéaire entre la résistance et la force. Dans l'implémentation actuelle on utilise un modèle de type "power-law" sur la résistance :

$$R_{jauge} = a \cdot Force^{-b}$$

Ce modèle a été retenu car il s'inverse simplement pour obtenir `Force = (a / R_{jauge})^{1/b}` et s'adapte bien aux résultats expérimentaux après calibration. Alternatives possibles : modèle linéaire, polynôme d'ordre supérieur ou table de correspondance (LUT) — chacun a ses compromis entre complexité et précision.

### 3.3 Stabilité et Lissage

Le système applique une **moyenne mobile sur 10 échantillons** :
- Chaque échantillon = mesure toutes les 100ms
- Envoi = moyenne sur 1 seconde (10 × 100ms)

**Formule :**
$$\text{valeur affichée} = \frac{1}{10} \sum_{i=0}^{9} \text{weight}_i$$

**Avantage :** Réduit le bruit ADC de ~√10 ≈ 3x.

---

## Tests Automatiques

### 4.1 Architecture du Test

L'autotest est encapsulé dans le composant `autotest/` et peut être activé au démarrage en décommentant :
```c
// Dans main.c
// startAutoTest(&components);
```

### 4.2 Séquence de Test

La fonction `startAutoTest()` exécute les tests dans cet ordre :

#### **1. Test Écran LCD (2 secondes)**
```c
static void startLcdTest(autotest_t *componants) {
    const char *first_line = "EPITA  2025/2026";
    const char *second_line = "VASSEUR,JOUY,OLIVER";
    lcd_defil_name(componants->lcd, first_line, second_line);
}
```
- Affiche les noms des auteurs avec défilement
- Vérifie la communication I2C et l'affichage

#### **2. Test LEDs (2 secondes)**
```c
static void startLedTest(autotest_t *componants) {
    lcd_clear(componants->lcd);
    lcd_print(componants->lcd, "Test des LEDs\nD1 et D6");
    
    led_blink(componants->led1, 5, 500);
    vTaskDelay(1 * SECONDS);
    led_blink(componants->led2, 5, 500);
}
```
- LED1 clignote 5 fois à 500ms de période
- Attente 1 seconde
- LED2 clignote 5 fois à 500ms de période

#### **3. Test Buzzer (1 seconde)**
```c
static void startBuzzerTest(autotest_t *componants) {
    lcd_clear(componants->lcd);
    lcd_print(componants->lcd, "Test du Buzzer");
    buzzer_off(componants->buzzer);
}
```
- Affiche le message sur l'écran
- Vérifie que le buzzer peut être commandé

#### **4. Test Boutons (attente interactive)**
```c
static void ButtonTest(autotest_t *componants, button_t *button) {
    TickType_t start_time = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(10000);  // 10 secondes timeout
    
    while ((xTaskGetTickCount() - start_time) < timeout) {
        if (button_is_pressed(button)) {
            lcd_print_line(componants->lcd, "Touche OK", 1);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    lcd_print_line(componants->lcd, "Touche KO", 1);
}
```
- Attend que l'utilisateur appuie sur le bouton (timeout 10s)
- Affiche "OK" ou "KO"
- Teste les deux boutons séquentiellement

#### **5. Test Capteur de Poids (3 secondes)**
```c
static void startPressureTest(autotest_t *componants) {
    lcd_clear(componants->lcd);
    lcd_print(componants->lcd, "Test du Capteur\nde Poids");
    
    pressure_tare();  // Remise à zéro
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 10 lectures et affichage sur console
    for (size_t i = 0; i < 10; i++) {
        int raw = pressure_read_raw();
        float v = pressure_read_voltage();
        float w = pressure_read_weight();
        printf("ADC=%d | V=%.2fV | W=%.1fg\n", raw, v, w);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```
- Tarage du capteur
- 10 lectures successives affichées sur le port série

### 4.3 Résultats Attendus

| Test | Résultat Attendu |
|------|-----------------|
| LCD | Affichage des noms sans scintillation |
| LED | Chaque LED clignote 5 fois distinctement |
| Buzzer | Pas d'erreur de communication |
| Boutons | "OK" si appui détecté, "KO" si timeout |
| Capteur | Valeurs stables (ADC, V, poids) affichées |

---

## Justificatifs de Conception

### 5.1 Choix de FreeRTOS

**Justification :**
- **Temps réel :** FreeRTOS permet de garantir des délais d'exécution prévisibles pour chaque composant
- **Multitâche :** Chaque capteur s'exécute indépendamment sans bloquer les autres
- **Intégration ESP-IDF :** FreeRTOS est le noyau natif d'ESP-IDF


### 5.2 Queues de Messages vs Appels Directs

| Aspect | Queues | Appels Directs |
|--------|--------|----------------|
| **Couplage** | Faible (découplage temporel) | Fort |
| **Scalabilité** | Facile d'ajouter consommateurs | Complexe |
| **Blocage** | Non-bloquant possible | Potentiellement bloquant |
| **Débogage** | Traçabilité des messages | Implicite |

**Choix :** Queues pour garantir une architecture évolutive.

### 5.3 Dédoublement des Lectures Bouton

```
Sans dédoublement:
Appui réel: ___|‾‾‾|___
Bruit:     ___|‾‾‾|‾|_|___
Détection: 3-4 événements au lieu de 1

Avec dédoublement (50ms):
Apui + attente 50ms + deuxième lecture
= filtre analogique logiciel naturel
```

**Coût :** 50ms de latence négligeable pour des boutons.

### 5.4 Modèle Power-Law pour la Jauge de Contrainte

**Justification mathématique :**

La jauge de contrainte suit une relation empirique de type "power-law" entre la résistance et la force appliquée :

$$R_{jauge} = a \cdot Force^{-b}$$

où $a$ (coefficient `CALIBRATION_COEF_A`, défaut 10000) et $b$ (coefficient `CALIBRATION_COEF_B`, défaut 0.7) sont déterminés lors de la calibration.

**Inversion du modèle :**

Pour estimer la force à partir de la résistance mesurée, on inverse l'équation :

$$Force = \left(\frac{a}{R_{jauge}}\right)^{1/b}$$

**Chaîne de conversion complète :**

1. **ADC brut** (0–4095) → `convert_adc_to_voltage()` → **Tension mesurée** (V)
2. **Tension mesurée** → `calculate_r_jauge()` via pont diviseur inverse → **Résistance jauge** (Ω)
3. **Résistance jauge** → `calculate_force_from_r_jauge()` via inversion du modèle → **Poids brut**
4. **Poids brut** × 100 − `CALIBRATION_OFFSET_G` → **Poids final** (g)

**Avantages du modèle power-law :**
- S'adapte bien aux courbes réelles des jauges de contrainte
- Simple à inverser (une seule formule)
- Facile à calibrer : mesurer la résistance à quelques points de référence et ajuster $a$ et $b$
- Coefficients constants et mémorisables

**Paramètres de calibration :**
- `CALIBRATION_COEF_A` (défaut 10000) : coefficient de la courbe
- `CALIBRATION_COEF_B` (défaut 0.7) : exposant de la courbe
- `CALIBRATION_OFFSET_G` : offset de tarage mémorisé en NVM via `pressure_tare()`
- Paramètres matériel : `V_SUPPLY` (4.8V), `R_M` (10 kΩ), `V_REF` (5V)

### 5.5 Taille des Buffers

| Buffer | Taille | Justification |
|--------|--------|--------------|
| `msg_q_sensor` | 20 messages | Absorption des pics d'événements |
| `msg_q_lcd` | 20 messages | Synchronisation avec I2C lent |
| `values[]` (poids) | 10 échantillons | Lissage sans latence excessive |

Exemple : Si 3 événements arrivent simultanément (pression + 2 boutons) + affichage LCD retardé, la queue de 20 absorbe sans perte.

### 5.6 Priorités des Tâches

```c
xTaskCreate(lcd_task, "lcd_task", 2048, &lcd, 5, NULL);
xTaskCreate(button_task, "button1_task", 2048, &button1, 5, NULL);
xTaskCreate(button_task, "button2_task", 2048, &button2, 5, NULL);
xTaskCreate(pressure_task, "pressure_task", 2048, &pressure_sensor, 5, NULL);
```

Toutes les tâches périphériques à **priorité 5** (identique).
- Évite les inversions de priorité
- FreeRTOS arbitre le scheduling équitablement
- app_main contrôle la logique métier avec queue bloquante

### 5.7 Accumulation d'Énergie (Veille)

```c
uint8_t veille_state = 1;
while (1) {
    xQueueReceive(msg_q_sensor, &msg, portMAX_DELAY);  // Bloqué si rien
    
    if (msg.id == button1.id && msg.value == 1) {
        veille_state = !veille_state;  // Toggle veille
    }
    
    if (veille_state) {
        xQueueSend(msg_q_lcd, &msg, portMAX_DELAY);  // Afficher si actif
    }
}
```

**Logique :**
- Bouton 1 = activation/désactivation de l'écran
- Bouton 2 = tarage du capteur
- Mode veille économise la batterie en désactivant l'affichage

---

## Synthèse et Perspectives

### Points Forts de l'Architecture
**Modularité** : Chaque composant est indépendant et testable  
**Scalabilité** : Facile d'ajouter de nouveaux capteurs (queue + tâche)  
**Robustesse** : Dédoublement des lectures, calibrage adapté  
**Performance** : Pas de blocages critiques, latences prévisibles  
**Maintenabilité** : Code bien structuré, interfaces claires  

### Améliorations Futures Possibles
- **Persistence NVM** : Mémorisation des paramètres de calibrage

---

**Auteurs :** VASSEUR
**Date :** 2025/2026  
