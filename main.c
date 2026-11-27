/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <math.h>

/* hUsbDeviceFS se define en usb_device.c pero su header no lo declara */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ============================================================
   ORDENADOR DE PINTURAS - 3 PID de posición (grados)
   STM32 Black Pill (F411/F401) - Keil MDK + STM32 HAL (CubeMX)

   Motores: 0 = Base (columna), 1 = Hombro, 2 = Brazo
   Drivers: 3 x A4988, microstep 1/16

   PINES (GPIOA, configurados por este archivo):
     STEP: PA1, PA3, PA5     DIR: PA2, PA4, PA6     ENABLE: PA7
     LED de latido: PC13

   PROTOCOLO SERIAL (USB CDC, líneas terminadas en '\n'):
   PC -> STM32
     R,a0,a1,a2        referencias en grados
     P,i,kp,ki,kd      PID del motor i (0..2)
     F,tau             filtro de referencia (s), 0 = sin filtro
     Z                 poner posición actual como 0°
     X                 parar y mantener posición actual
     E,1 / E,0         habilitar / deshabilitar drivers
   STM32 -> PC
     D,ms,r0,y0,r1,y1,r2,y2   cada 100 ms (r = objetivo, y = medido, en °)
     #texto                   mensajes informativos
   ============================================================ */

#define NJ 3

/* ---------- PINES (todos en GPIOA) ---------- */
#define PORT      GPIOA
#define EN_MASK   GPIO_PIN_7
static const uint16_t STEP_MASK[NJ] = {GPIO_PIN_1, GPIO_PIN_3, GPIO_PIN_5};
static const uint16_t DIR_MASK[NJ]  = {GPIO_PIN_2, GPIO_PIN_4, GPIO_PIN_6};

/* ---------- MECÁNICA (ajustar a tu brazo) ---------- */
#define PASOS_REV   (200.0f * 16.0f)                 /* NEMA 1.8° con 1/16            */
static const float  REDUCCION[NJ] = {1.0f, 1.0f, 1.0f};  /* vueltas motor / vuelta art. */
static const int8_t SENTIDO[NJ]   = {1, 1, 1};           /* -1 para invertir el giro    */
static const float  ANG_MIN[NJ]   = {-180.0f, -90.0f, -120.0f}; /* iguales en la GUI    */
static const float  ANG_MAX[NJ]   = { 180.0f,  90.0f,  120.0f};
#define FREQ_MAX    3000.0f                          /* pasos/s máx. por motor        */
#define FREQ_MIN    10.0f                            /* pasos/s mínimos (zona muerta) */

static float pasosPorGrado[NJ];
static float uMax[NJ];                               /* °/s máx (según FREQ_MAX)      */

/* ---------- ESTADO DE LOS MOTORES ---------- */
static volatile int32_t pos[NJ]   = {0, 0, 0};       /* pasos acumulados              */
static int8_t   dirN[NJ]          = {1, 1, 1};
static uint32_t periodo_cyc[NJ]   = {0, 0, 0};       /* en ciclos de CPU, 0 = parado  */
static uint32_t t_paso[NJ]        = {0, 0, 0};
static uint8_t  habilitado        = 1;

/* ---------- PID (valores iniciales; la GUI los reemplaza) ---------- */
static float Kp[NJ] = {3.0f, 3.0f, 3.0f};
static float Ki[NJ] = {0.5f, 0.5f, 0.5f};
static float Kd[NJ] = {0.0f, 0.0f, 0.0f};

static float refObj[NJ] = {0, 0, 0};                 /* objetivo (°)                  */
static float r_n[NJ]    = {0, 0, 0};                 /* referencia filtrada           */
static float y_n[NJ]    = {0, 0, 0};                 /* posición medida (°)           */
static float e_n1[NJ]   = {0, 0, 0};
static float ei[NJ]     = {0, 0, 0};

/* ---------- FILTRO DE REFERENCIA ---------- */
#define TS_S  0.01f                                  /* 10 ms                         */
static float polo  = 0.0f;
static float aFilt = 1.0f;

static void setTau(float tau)
{
    if (tau <= 0.0f) polo = 0.0f;
    else             polo = expf(-TS_S / tau);
    aFilt = 1.0f - polo;
}

/* ---------- TEMPORIZADOR TIM3 (10 ms, registros directos) ---------- */
static volatile uint8_t timer_flag = 0;
static uint8_t contTx = 0;

void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR = (uint32_t)~TIM_SR_UIF;
        timer_flag = 1;
    }
}

static void timer_init(void)
{
    uint32_t pclk1  = HAL_RCC_GetPCLK1Freq();
    uint32_t timclk = ((RCC->CFGR & RCC_CFGR_PPRE1_2) == 0U) ? pclk1 : (pclk1 * 2U);

    __HAL_RCC_TIM3_CLK_ENABLE();
    TIM3->PSC = (timclk / 1000000U) - 1U;            /* 1 MHz                          */
    TIM3->ARR = 10000U - 1U;                         /* 10 ms                          */
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR  = 0U;
    TIM3->DIER = TIM_DIER_UIE;
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 = TIM_CR1_CEN;
}

/* ---------- CONTADOR DE CICLOS (DWT) PARA LOS PASOS ---------- */
static uint32_t cyc_2us;

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    cyc_2us = (SystemCoreClock / 1000000U) * 2U;
}

/* ---------- COLA TX (USB CDC) ---------- */
#define TX_SZ 1024U
static uint8_t  txbuf[TX_SZ];
static uint16_t txh = 0, txt = 0;

static uint16_t tx_free(void)
{
    return (uint16_t)((txt + TX_SZ - txh - 1U) % TX_SZ);
}

static void tx_char(char c)
{
    uint16_t nh = (uint16_t)((txh + 1U) % TX_SZ);
    if (nh != txt) {                                 /* si está llena, descarta */
        txbuf[txh] = (uint8_t)c;
        txh = nh;
    }
}

static void tx_str(const char *s)
{
    while (*s) tx_char(*s++);
}

static void tx_u32(uint32_t v)
{
    char tmp[11];
    int n = 0;
    if (v == 0U) { tx_char('0'); return; }
    while (v > 0U) { tmp[n++] = (char)('0' + (v % 10U)); v /= 10U; }
    while (n > 0) tx_char(tmp[--n]);
}

static void tx_f1(float v)                           /* un decimal, sin printf */
{
    int32_t x = (int32_t)((v >= 0.0f) ? (v * 10.0f + 0.5f) : (v * 10.0f - 0.5f));
    if (x < 0) { tx_char('-'); x = -x; }
    tx_u32((uint32_t)(x / 10));
    tx_char('.');
    tx_char((char)('0' + (x % 10)));
}

static void tx_pump(void)
{
    static uint8_t chunk[64];                        /* debe persistir hasta TxCplt */
    uint16_t n = 0, t = txt;

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        txh = txt = 0;                               /* sin host: vaciar cola       */
        return;
    }
    if (txh == txt) return;

    while (t != txh && n < sizeof(chunk)) {
        chunk[n++] = txbuf[t];
        t = (uint16_t)((t + 1U) % TX_SZ);
    }
    if (CDC_Transmit_FS(chunk, n) == USBD_OK) {
        txt = t;
    }
}

/* ---------- COLA RX (se llena desde la interrupción USB) ---------- */
#define RX_SZ 256U
static volatile uint8_t  rxbuf[RX_SZ];
static volatile uint16_t rxh = 0, rxt = 0;

void App_RxBytes(uint8_t *buf, uint32_t len)
{
    uint32_t i;
    for (i = 0; i < len; i++) {
        uint16_t nh = (uint16_t)((rxh + 1U) % RX_SZ);
        if (nh != rxt) {
            rxbuf[rxh] = buf[i];
            rxh = nh;
        }
    }
}

/* ---------- PARSEO ---------- */
static float parse_f(const char *s)
{
    float sign = 1.0f, v = 0.0f;
    uint32_t frac = 0U;
    float div = 1.0f;

    if (*s == '-') { sign = -1.0f; s++; }
    else if (*s == '+') { s++; }
    while (*s >= '0' && *s <= '9') { v = v * 10.0f + (float)(*s - '0'); s++; }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9' && div < 1.0e8f) {
            frac = frac * 10U + (uint32_t)(*s - '0');
            div *= 10.0f;
            s++;
        }
        v += (float)frac / div;
    }
    return sign * v;
}

static int parse_i(const char *s)
{
    int sign = 1, v = 0;
    if (*s == '-') { sign = -1; s++; }
    while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); s++; }
    return sign * v;
}

static float limitar(float v, uint8_t i)
{
    if (v < ANG_MIN[i]) return ANG_MIN[i];
    if (v > ANG_MAX[i]) return ANG_MAX[i];
    return v;
}

static void mantenerPosicion(void)
{
    uint8_t i;
    for (i = 0; i < NJ; i++) {
        refObj[i] = y_n[i];
        r_n[i]    = y_n[i];
        ei[i]     = 0.0f;
        e_n1[i]   = 0.0f;
    }
}

static void procesar(char *s)
{
    char *f[6];
    int nf = 0;
    char *p;
    uint8_t i;

    f[nf++] = s;
    for (p = s; *p; p++) {
        if (*p == ',') {
            *p = 0;
            if (nf < 6) f[nf++] = p + 1;
        }
    }

    switch (f[0][0]) {
    case 'R':
        if (nf < 4) { tx_str("#ERR R\n"); return; }
        for (i = 0; i < NJ; i++) refObj[i] = limitar(parse_f(f[1 + i]), i);
        break;

    case 'P': {
        int k;
        if (nf < 5) return;
        k = parse_i(f[1]);
        if (k < 0 || k >= NJ) { tx_str("#ERR P\n"); return; }
        Kp[k] = parse_f(f[2]);
        Ki[k] = parse_f(f[3]);
        Kd[k] = parse_f(f[4]);
        ei[k] = 0.0f;
        tx_str("#PID ");
        tx_u32((uint32_t)k);
        tx_char('\n');
        break;
    }

    case 'F':
        if (nf < 2) return;
        setTau(parse_f(f[1]));
        break;

    case 'Z':
        for (i = 0; i < NJ; i++) {
            pos[i] = 0; y_n[i] = 0.0f; refObj[i] = 0.0f; r_n[i] = 0.0f;
            ei[i] = 0.0f; e_n1[i] = 0.0f; periodo_cyc[i] = 0U;
        }
        tx_str("#CERO\n");
        break;

    case 'X':
        mantenerPosicion();
        tx_str("#PARADO\n");
        break;

    case 'E':
        if (nf < 2) return;
        if (parse_i(f[1]) == 1) {
            mantenerPosicion();
            habilitado = 1;
            PORT->BSRR = ((uint32_t)EN_MASK) << 16;          /* EN = LOW  */
            tx_str("#HABILITADO\n");
        } else {
            habilitado = 0;
            PORT->BSRR = (uint32_t)EN_MASK;                  /* EN = HIGH */
            for (i = 0; i < NJ; i++) periodo_cyc[i] = 0U;
            tx_str("#DESHABILITADO\n");
        }
        break;

    default:
        break;
    }
}

static void rx_service(void)
{
    static char line[80];
    static uint8_t n = 0;

    while (rxt != rxh) {
        char c = (char)rxbuf[rxt];
        rxt = (uint16_t)((rxt + 1U) % RX_SZ);

        if (c == '\n' || c == '\r') {
            if (n > 0U) {
                line[n] = 0;
                procesar(line);
                n = 0U;
            }
        } else if (n < (sizeof(line) - 1U)) {
            line[n++] = c;
        }
    }
}

/* ============================================================
   INICIALIZACIÓN
   ============================================================ */
void App_Init(void)
{
    GPIO_InitTypeDef g = {0};
    uint8_t i;
    uint16_t all = EN_MASK;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    for (i = 0; i < NJ; i++) all |= (uint16_t)(STEP_MASK[i] | DIR_MASK[i]);

    g.Pin   = all;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PORT, &g);
    PORT->BSRR = ((uint32_t)all) << 16;              /* todo en LOW (EN=LOW: habilitado) */

    g.Pin   = GPIO_PIN_13;                           /* LED Black Pill (activo en bajo)  */
    HAL_GPIO_Init(GPIOC, &g);
    GPIOC->BSRR = GPIO_PIN_13;

    for (i = 0; i < NJ; i++) {
        pasosPorGrado[i] = PASOS_REV * REDUCCION[i] / 360.0f;
        uMax[i]          = FREQ_MAX / pasosPorGrado[i];
    }

    setTau(0.3f);
    dwt_init();
    timer_init();
}

/* ============================================================
   LAZO PRINCIPAL (llamar continuamente desde while(1))
   ============================================================ */
void App_Loop(void)
{
    uint8_t i;

    /* ---------- GENERADOR DE PASOS (3 motores) ---------- */
    uint32_t ahora = DWT->CYCCNT;
    for (i = 0; i < NJ; i++) {
        if (periodo_cyc[i] > 0U && (uint32_t)(ahora - t_paso[i]) >= periodo_cyc[i]) {
            uint32_t t0;
            t_paso[i] = ahora;
            PORT->BSRR = (uint32_t)STEP_MASK[i];                   /* STEP = HIGH */
            t0 = DWT->CYCCNT;
            while ((uint32_t)(DWT->CYCCNT - t0) < cyc_2us) { }     /* >= 1 us     */
            PORT->BSRR = ((uint32_t)STEP_MASK[i]) << 16;           /* STEP = LOW  */
            pos[i] += dirN[i];
        }
    }

    /* ---------- COMANDOS DEL PC ---------- */
    rx_service();

    /* ---------- LAZO DE CONTROL cada 10 ms ---------- */
    if (timer_flag) {
        timer_flag = 0;

        for (i = 0; i < NJ; i++) {
            float e, ed, u, freq;
            uint8_t s1 = 0U, s2;

            /* Posición medida (conteo de pasos) */
            y_n[i] = (float)pos[i] / pasosPorGrado[i];

            /* Drivers deshabilitados: no acumular error */
            if (!habilitado) {
                refObj[i] = y_n[i];
                r_n[i]    = y_n[i];
                ei[i]     = 0.0f;
                e_n1[i]   = 0.0f;
                periodo_cyc[i] = 0U;
                continue;
            }

            /* Filtro de referencia */
            r_n[i] = aFilt * refObj[i] + polo * r_n[i];

            /* Error y derivada */
            e  = r_n[i] - y_n[i];
            ed = (e - e_n1[i]) / TS_S;
            e_n1[i] = e;

            /* PID */
            u = Kp[i] * e + Ki[i] * ei[i] + Kd[i] * ed;

            /* Saturación simétrica */
            if (u >= uMax[i])       { u =  uMax[i]; s1 = 1U; }
            else if (u <= -uMax[i]) { u = -uMax[i]; s1 = 1U; }

            /* Anti-windup (integración condicional) */
            s2 = (uint8_t)(((e > 0.0f && u > 0.0f) || (e < 0.0f && u < 0.0f)) ? 1U : 0U);
            if ((s1 & s2) == 0U) {
                ei[i] += e * TS_S;
            }

            /* u (°/s) -> dirección y frecuencia de pasos */
            freq = fabsf(u) * pasosPorGrado[i];
            if (freq < FREQ_MIN) {
                periodo_cyc[i] = 0U;
            } else {
                dirN[i] = (u >= 0.0f) ? 1 : -1;
                if (dirN[i] * SENTIDO[i] > 0) PORT->BSRR = (uint32_t)DIR_MASK[i];
                else                          PORT->BSRR = ((uint32_t)DIR_MASK[i]) << 16;
                periodo_cyc[i] = (uint32_t)((float)SystemCoreClock / freq);
            }
        }

        /* Telemetría cada 100 ms */
        if (++contTx >= 10U) {
            contTx = 0U;
            GPIOC->ODR ^= GPIO_PIN_13;                             /* latido */
            if (tx_free() > 90U) {
                tx_str("D,");
                tx_u32(HAL_GetTick());
                for (i = 0; i < NJ; i++) {
                    tx_char(',');
                    tx_f1(refObj[i]);
                    tx_char(',');
                    tx_f1(y_n[i]);
                }
                tx_char('\n');
            }
        }
    }

    /* ---------- ENVÍO USB ---------- */
    tx_pump();
}

/* ---------- RECEPCIÓN USB SIN EDITAR usbd_cdc_if.c ---------- */
static USBD_CDC_ItfTypeDef fops_hook;

static int8_t Rx_Hook(uint8_t *Buf, uint32_t *Len)
{
    App_RxBytes(Buf, *Len);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return (USBD_OK);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  fops_hook = USBD_Interface_fops_FS;
  fops_hook.Receive = Rx_Hook;
  USBD_CDC_RegisterInterface(&hUsbDeviceFS, &fops_hook);
  App_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_Loop();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4
                           PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
