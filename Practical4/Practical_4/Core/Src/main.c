/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f4xx.h"
#include "lcd_stm32f4.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables

#define NS 256 // Number of samples in LUT
#define TIM2CLK 16000000 // STM Clock frequency: Hint You might want to check the ioc file

// TIM2_Ticks = TIM2CLK / (NS * Fsignal).
// With NS=256, TIM2CLK=16 MHz, and Fsignal=1 kHz → TIM2_Ticks ≈ 63.
// This gives enough timer ticks between DMA transfers for stable operation
// while still producing an audible tone (~1 kHz).

#define F_SIGNAL 1000 // Frequency of output analog signal



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs

static const uint32_t Sin_LUT[NS] = {
    2048, 2098, 2148, 2198, 2248, 2298, 2348, 2398,
    2447, 2497, 2546, 2595, 2642, 2690, 2737, 2783,
    2831, 2875, 2923, 2966, 3013, 3054, 3100, 3141,
    3185, 3222, 3267, 3303, 3346, 3380, 3423, 3454,
    3495, 3525, 3565, 3590, 3630, 3653, 3692, 3713,
    3750, 3769, 3804, 3821, 3853, 3868, 3898, 3911,
    3939, 3949, 3975, 3984, 4007, 4014, 4034, 4040,
    4056, 4060, 4073, 4076, 4085, 4088, 4093, 4094,
    4095, 4094, 4093, 4088, 4085, 4076, 4073, 4060,
    4056, 4040, 4034, 4014, 4007, 3984, 3975, 3949,
    3939, 3911, 3898, 3868, 3853, 3821, 3804, 3769,
    3750, 3713, 3692, 3653, 3630, 3590, 3565, 3525,
    3495, 3454, 3423, 3380, 3346, 3303, 3267, 3222,
    3185, 3141, 3100, 3054, 3013, 2966, 2923, 2875,
    2831, 2783, 2737, 2690, 2642, 2595, 2546, 2497,
    2447, 2398, 2348, 2298, 2248, 2198, 2148, 2098,
    2048, 1997, 1947, 1897, 1847, 1797, 1747, 1697,
    1648, 1598, 1549, 1500, 1453, 1405, 1358, 1312,
    1264, 1220, 1172, 1129, 1082, 1041,  995,  954,
     910,  873,  828,  792,  749,  715,  672,  641,
     600,  570,  530,  505,  465,  442,  403,  382,
     345,  326,  291,  274,  242,  227,  197,  184,
     156,  146,  120,  111,   88,   81,   61,   55,
      39,   35,   22,   19,   10,    7,    2,    1,
       0,    1,    2,    7,   10,   19,   22,   35,
      39,   55,   61,   81,   88,  111,  120,  146,
     156,  184,  197,  227,  242,  274,  291,  326,
     345,  382,  403,  442,  465,  505,  530,  570,
     600,  641,  672,  715,  749,  792,  828,  873,
     910,  954,  995, 1041, 1082, 1129, 1172, 1220,
    1264, 1312, 1358, 1405, 1453, 1500, 1549, 1598,
    1648, 1697, 1747, 1797, 1847, 1897, 1947, 1997
};

static const uint32_t Saw_LUT[NS] = {
       0,   16,   32,   48,   64,   80,   96,  112,
     128,  144,  160,  176,  192,  208,  224,  240,
     256,  272,  288,  304,  320,  336,  352,  368,
     384,  400,  416,  432,  448,  464,  480,  496,
     512,  528,  544,  560,  576,  592,  608,  624,
     640,  656,  672,  688,  704,  720,  736,  752,
     768,  784,  800,  816,  832,  848,  864,  880,
     896,  912,  928,  944,  960,  976,  992, 1008,
    1024, 1040, 1056, 1072, 1088, 1104, 1120, 1136,
    1152, 1168, 1184, 1200, 1216, 1232, 1248, 1264,
    1280, 1296, 1312, 1328, 1344, 1360, 1376, 1392,
    1408, 1424, 1440, 1456, 1472, 1488, 1504, 1520,
    1536, 1552, 1568, 1584, 1600, 1616, 1632, 1648,
    1664, 1680, 1696, 1712, 1728, 1744, 1760, 1776,
    1792, 1808, 1824, 1840, 1856, 1872, 1888, 1904,
    1920, 1936, 1952, 1968, 1984, 2000, 2016, 2032,
    2048, 2064, 2080, 2096, 2112, 2128, 2144, 2160,
    2176, 2192, 2208, 2224, 2240, 2256, 2272, 2288,
    2304, 2320, 2336, 2352, 2368, 2384, 2400, 2416,
    2432, 2448, 2464, 2480, 2496, 2512, 2528, 2544,
    2560, 2576, 2592, 2608, 2624, 2640, 2656, 2672,
    2688, 2704, 2720, 2736, 2752, 2768, 2784, 2800,
    2816, 2832, 2848, 2864, 2880, 2896, 2912, 2928,
    2944, 2960, 2976, 2992, 3008, 3024, 3040, 3056,
    3072, 3088, 3104, 3120, 3136, 3152, 3168, 3184,
    3200, 3216, 3232, 3248, 3264, 3280, 3296, 3312,
    3328, 3344, 3360, 3376, 3392, 3408, 3424, 3440,
    3456, 3472, 3488, 3504, 3520, 3536, 3552, 3568,
    3584, 3600, 3616, 3632, 3648, 3664, 3680, 3696,
    3712, 3728, 3744, 3760, 3776, 3792, 3808, 3824,
    3840, 3856, 3872, 3888, 3904, 3920, 3936, 3952,
    3968, 3984, 4000, 4016, 4032, 4048, 4064, 4080
};

static const uint32_t Triangle_LUT[NS] = {
       0,   32,   64,   96,  128,  160,  192,  224,
     256,  288,  320,  352,  384,  416,  448,  480,
     512,  544,  576,  608,  640,  672,  704,  736,
     768,  800,  832,  864,  896,  928,  960,  992,
    1024, 1056, 1088, 1120, 1152, 1184, 1216, 1248,
    1280, 1312, 1344, 1376, 1408, 1440, 1472, 1504,
    1536, 1568, 1600, 1632, 1664, 1696, 1728, 1760,
    1792, 1824, 1856, 1888, 1920, 1952, 1984, 2016,
    2048, 2080, 2112, 2144, 2176, 2208, 2240, 2272,
    2304, 2336, 2368, 2400, 2432, 2464, 2496, 2528,
    2560, 2592, 2624, 2656, 2688, 2720, 2752, 2784,
    2816, 2848, 2880, 2912, 2944, 2976, 3008, 3040,
    3072, 3104, 3136, 3168, 3200, 3232, 3264, 3296,
    3328, 3360, 3392, 3424, 3456, 3488, 3520, 3552,
    3584, 3616, 3648, 3680, 3712, 3744, 3776, 3808,
    3840, 3872, 3904, 3936, 3968, 4000, 4032, 4064,
    4095, 4064, 4032, 4000, 3968, 3936, 3904, 3872,
    3840, 3808, 3776, 3744, 3712, 3680, 3648, 3616,
    3584, 3552, 3520, 3488, 3456, 3424, 3392, 3360,
    3328, 3296, 3264, 3232, 3200, 3168, 3136, 3104,
    3072, 3040, 3008, 2976, 2944, 2912, 2880, 2848,
    2816, 2784, 2752, 2720, 2688, 2656, 2624, 2592,
    2560, 2528, 2496, 2464, 2432, 2400, 2368, 2336,
    2304, 2272, 2240, 2208, 2176, 2144, 2112, 2080,
    2048, 2016, 1984, 1952, 1920, 1888, 1856, 1824,
    1792, 1760, 1728, 1696, 1664, 1632, 1600, 1568,
    1536, 1504, 1472, 1440, 1408, 1376, 1344, 1312,
    1280, 1248, 1216, 1184, 1152, 1120, 1088, 1056,
    1024,  992,  960,  928,  896,  864,  832,  800,
     768,  736,  704,  672,  640,  608,  576,  544,
     512,  480,  448,  416,  384,  352,  320,  288,
     256,  224,  192,  160,  128,   96,   64,   32
};

static const uint32_t Piano_LUT[256] = {1896, 1896, 1896, 1896, 1898, 1895, 1895, 1016, 1536, 13, 1379, 2340, 1611, 2374, 1274, 2537, 3493, 2371, 2806, 1811, 2184, 2798, 1684, 1666, 1356, 1357, 1649, 1333, 1357, 1565, 1696, 1582, 2007, 2165, 2256, 2467, 1821, 2338, 2470, 2082, 2335, 1390, 1789, 2131, 1528, 1880, 1397, 1673, 2200, 1741, 1897, 1811, 1904, 2244, 2006, 1919, 2023, 1963, 1941, 1978, 1859, 1908, 1953, 1572, 1859, 1902, 1796, 1982, 1553, 1881, 2116, 1861, 2043, 1722, 1917, 2153, 1925, 1971, 1822, 1902, 2039, 1887, 1856, 1802, 1834, 1845, 1872, 1870, 1851, 1925, 1833, 1925, 1970, 1932, 1988, 1802, 1903, 1965, 1920, 1952, 1781, 1874, 1947, 1916, 1927, 1809, 1878, 1925, 1905, 1888, 1859, 1911, 1905, 1918, 1908, 1897, 1939, 1869, 1887, 1905, 1870, 1910, 1861, 1874, 1893, 1888, 1926, 1879, 1917, 1927, 1878, 1928, 1907, 1913, 1937, 1902, 1913, 1898, 1868, 1912, 1886, 1866, 1881, 1847, 1891, 1901, 1886, 1891, 1908, 1917, 1919, 1921, 1893, 1908, 1904, 1888, 1898, 1851, 1882, 1764, 1871, 1935, 1898, 1934, 1841, 1949, 1998, 1938, 1971, 1888, 1925, 1959, 1874, 1888, 1855, 1847, 1869, 1842, 1840, 1873, 1895, 1880, 1920, 1902, 1929, 1953, 1883, 1942, 1929, 1900, 1933, 1852, 1891, 1930, 1860, 1890, 1863, 1872, 1935, 1888, 1886, 1891, 1893, 1923, 1914, 1899, 1891, 1901, 1876, 1893, 1896, 1879, 1894, 1872, 1894, 1911, 1893, 1900, 1883, 1910, 1917, 1908, 1912, 1889, 1903, 1915, 1901, 1910, 1885, 1884, 1883, 1875, 1718, 2925, 1432, 1873, 1460, 1074, 2092, 2684, 2706, 1725, 1333, 1168, 2546, 2171, 2393, 1885, 1289, 1858, 1378, 2383, 2275, 1979, 1538, 1412};

static const uint32_t Guitar_LUT[256] = {1966, 1966, 1966, 1966, 1966, 1966, 1966, 1991, 1969, 2007, 1806, 1819, 2775, 1739, 1178, 2783, 1955, 1740, 1620, 2422, 1902, 1941, 1738, 2238, 2182, 1470, 2310, 1973, 1854, 1664, 2550, 1695, 1811, 2026, 2192, 1988, 1561, 2305, 1921, 2002, 1559, 2410, 1872, 1807, 1947, 2193, 1987, 1654, 2248, 1865, 2066, 1596, 2339, 1870, 1873, 1893, 2165, 2044, 1658, 2179, 1917, 2093, 1576, 2286, 1926, 1889, 1854, 2156, 2064, 1706, 2135, 1911, 2142, 1599, 2231, 1948, 1942, 1800, 2138, 2084, 1714, 2099, 1930, 2154, 1612, 2189, 1968, 1979, 1772, 2120, 2114, 1736, 2056, 1948, 2169, 1618, 2149, 1982, 1965, 1716, 2130, 2022, 1631, 2015, 2084, 2492, 1894, 2326, 1955, 1713, 1577, 1967, 2029, 1738, 1923, 1950, 2354, 1834, 2435, 2080, 1938, 1570, 1821, 2007, 1699, 1854, 2003, 2257, 1774, 2413, 2212, 2082, 1693, 1854, 1901, 1570, 2086, 1485, 2522, 1955, 2797, 2271, 1866, 2088, 2660, 3604, 648, 1313, 2482, 1466, 925, 2096, 2987, 2460, 1997, 2087, 2864, 1175, 383, 2238, 2014, 1628, 2094, 2406, 3101, 2166, 1594, 1865, 1580, 1225, 1708, 2233, 1924, 2525, 2131, 2880, 2091, 1506, 1761, 1860, 1726, 2080, 1999, 1895, 1952, 1967, 2124, 1932, 1892, 2019, 2049, 1894, 1901, 1962, 1990, 1970, 1919, 1977, 2048, 1964, 1918, 1998, 2001, 1951, 1889, 1957, 1997, 1948, 1918, 2003, 2018, 1970, 1952, 1989, 1988, 1953, 1945, 1941, 1981, 1933, 1955, 1960, 2000, 1974, 1965, 1977, 1987, 1968, 1930, 1961, 1963, 1974, 1961, 2004, 1978, 1981, 1961, 1943, 1961, 1950, 1949, 1924, 1974, 1984, 2007, 1968, 1987, 1956, 1948, 1961, 1934, 1961, 1952, 1981, 1965, 2008};

static const uint32_t Drum_LUT[256] = {2048, 2048, 2047, 2047, 2047, 2050, 2043, 1991, 2114, 4069, 2316, 181, 4086, 1715, 272, 1497, 2342, 4095, 2759, 1332, 0, 2500, 2712, 3126, 3109, 501, 1797, 2219, 2161, 2218, 2259, 2168, 1271, 1672, 2415, 2615, 2524, 1885, 1341, 1725, 2293, 2589, 2072, 1861, 2065, 1933, 1905, 1996, 2242, 2475, 1980, 1488, 1830, 2330, 2405, 2204, 1695, 1834, 2240, 2152, 1920, 1920, 2269, 2186, 1898, 1809, 2032, 2251, 2187, 1999, 1858, 2011, 2173, 2100, 1943, 2029, 2135, 2091, 1981, 1979, 2087, 2100, 2074, 2025, 1985, 2026, 2086, 2129, 2057, 1972, 2005, 2064, 2090, 2050, 2027, 2064, 2061, 1890, 2055, 4083, 2309, 173, 4068, 1749, 227, 1505, 2298, 4089, 2856, 1341, 2, 2482, 2705, 3167, 3106, 499, 1801, 2217, 2161, 2218, 2265, 2174, 1278, 1662, 2411, 2611, 2523, 1888, 1344, 1721, 2293, 2589, 2072, 1861, 2065, 1933, 1905, 1997, 2220, 2478, 1998, 1478, 1828, 2315, 2404, 2213, 1696, 1834, 2239, 2153, 1920, 1920, 2273, 2181, 1903, 1811, 2035, 2254, 2187, 2003, 1856, 2013, 2171, 2100, 1942, 2029, 2135, 2091, 1981, 1979, 2087, 2099, 2075, 2024, 1985, 2026, 2085, 2129, 2057, 1972, 2005, 2064, 2090, 2050, 2026, 2064, 2037, 1825, 0, 3038, 1859, 234, 3554, 2037, 2101, 2979, 1243, 1537, 1988, 1393, 2307, 1713, 2339, 2129, 1640, 2465, 1879, 1849, 2264, 1569, 1942, 2380, 1658, 2411, 2222, 1859, 2199, 2042, 1966, 2148, 1936, 2035, 2099, 1953, 2081, 2009, 2057, 2104, 2024, 2087, 2066, 1991, 2038, 2062, 2060, 2060, 2030, 2081, 2014, 1999, 2051, 2033, 2059, 2064, 2052, 2040, 2033, 2041, 2082, 2027, 2021, 2068, 2049, 2065, 2046, 2051};

volatile uint8_t  waveIndex = 0;  // which LUT we’re on
uint32_t lastButtonPress = 0; // for debounce timing

// array of LUT pointers, so button can cycle through them
const uint32_t *LUTs[] = {Sin_LUT, Saw_LUT, Triangle_LUT, Piano_LUT, Guitar_LUT, Drum_LUT};

// names to show on LCD
const char *LUTnames[] = {"Sine", "Sawtooth", "Triangle", "Piano", "Guitar", "Drum"};

// total number of waves (auto-updates if you add more)
#define NUM_WAVES (sizeof(LUTs)/sizeof(LUTs[0]))

// where DMA writes values (PWM duty register)
uint32_t DestAddress = (uint32_t)&(TIM3->CCR3);


/* USER CODE END PV */


// TODO: Equation to calculate TIM2_Ticks
// TIM2_Ticks = TIM2CLK / (NS * Fsignal)

uint32_t TIM2_Ticks = 63; // How often to write new LUT value


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void EXTI0_IRQHandler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  // TODO: Start TIM3 in PWM mode on channel 3

  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)
  {
      Error_Handler();
  }

  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1

  if (HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
      Error_Handler();
  }

  // TODO: Start DMA in IT mode on TIM2->CH1. Source is LUT and Dest is TIM3->CCR3; start with Sine LUT

  // (source = LUT in memory, dest = PWM register, length = NS samples)

  if (HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)Sin_LUT, DestAddress, NS) != HAL_OK)
  {
      Error_Handler();
  }

  // TODO: Write current waveform to LCD (Sine is the first waveform)

  MX_GPIO_Init();

    //LCD must be initialized after HAL_Init + MX_GPIO_Init
    init_LCD();              // driver init function
    lcd_command(CLEAR);      // clear display
    lcd_putstring("Sine");   // show first waveform name

    MX_DMA_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();

  // TODO: Enable DMA (start transfer from LUT to CCR)
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* TIM2_CH1 DMA Init */
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_tim2_ch1.Instance = DMA1_Stream5;
  hdma_tim2_ch1.Init.Channel = DMA_CHANNEL_3;         // TIM2_CH1 is on channel 3
  hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH; // Memory -> TIM3->CCR3
  hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;    // Peripheral address fixed
  hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;        // Memory address increments
  hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;            // Repeat LUT automatically
  hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_tim2_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK)
  {
      Error_Handler();
  }

  /* Link DMA handle to TIM2 handle */
  __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // -------------------------------
  // LCD pins configuration
  // -------------------------------
  // Configure PC14 (RS) and PC15 (E) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Configure PB8 (D4) and PB9 (D5) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure PA12 (D6) and PA15 (D7) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Set all LCD pins LOW initially
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_RESET);


  // -------------------------------
  // Button0 configuration (PA0)
  // -------------------------------
  GPIO_InitStruct.Pin = Button0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Interrupt on rising edge
  GPIO_InitStruct.Pull = GPIO_PULLUP;         // Use pull-up resistor
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Enable and set EXTI line 0 interrupt priority
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_IRQHandler(void){

	// clear the EXTI flag first (avoids retriggering)
    HAL_GPIO_EXTI_IRQHandler(Button0_Pin);

	// TODO: Debounce using HAL_GetTick()

    uint32_t now = HAL_GetTick(); // current time (ms since boot)
    if ((now - lastButtonPress) < 150) return; // ignore if <150 ms since last press
    lastButtonPress = now; // update last press time


	// TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with new LUT and re-enable transfer
	// HINT: Consider using C's "switch" function to handle LUT changes

    __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);// stop DMA requests from TIM2
    HAL_DMA_Abort_IT(&hdma_tim2_ch1); // fully stop the old transfer

    // switch to next waveform
    waveIndex = (waveIndex + 1) % NUM_WAVES; // wrap back to 0 at end

    // show new waveform name on LCD
    lcd_command(CLEAR);
    lcd_putstring((char*)LUTnames[waveIndex]);

    //restart DMA with new LUT

    if (HAL_DMA_Start_IT(&hdma_tim2_ch1,(uint32_t)LUTs[waveIndex], DestAddress,NS) != HAL_OK)
    {
            Error_Handler();
        }

    __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1); // kick DMA back on

}
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
#ifdef USE_FULL_ASSERT
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
