# Implementation Guide for Remaining Files

## Quick Reference: Standard Templates

### Template 1: Header File (.h) Structure

```c
/*******************************************************************************
 * @file    filename.h
 * @brief   One-line description of module purpose
 * @author  Vikas Yadav
 * @date    2020
 ******************************************************************************/

#ifndef _FILENAME_H_
#define _FILENAME_H_

#include "required_headers.h"

/****************************************************************************
 * Module Constants
 ****************************************************************************/

#define CONSTANT_NAME        value    /* Description with units */
#define ANOTHER_CONSTANT     value    /* Description */

/****************************************************************************
 * Type Definitions (if any)
 ****************************************************************************/

// Type definitions here

/****************************************************************************
 * Function Prototypes
 ****************************************************************************/

/**
 * @brief Brief one-line description
 *
 * Optional detailed description paragraph.
 *
 * @param[in] input - Description with units and range
 * @param[out] output - Description of output
 * @param[in,out] inout - Description of input/output parameter
 * @return Return value description (or "None" if void)
 *
 * @note Special conditions or limitations
 */
void function_name(type input, type *output, type *inout);

#endif /* _FILENAME_H_ */
```

### Template 2: Source File (.c) Structure

```c
/*******************************************************************************
 * @file    filename.c
 * @brief   Implementation description
 * @author  Vikas Yadav
 * @date    2020
 ******************************************************************************/

#include "filename.h"

/****************************************************************************
 * Private Constants
 ****************************************************************************/

/* Mathematical constants */
#define DOUBLE_ZERO          0.0      /* Zero value for double precision */
#define DOUBLE_ONE           1.0      /* Unity value for double precision */
#define DOUBLE_TWO           2.0      /* Two for doubling operations */
#define DOUBLE_HALF          0.5      /* Half value */
#define DOUBLE_QUARTER       0.25     /* Quarter value */

/* Angle constants in degrees */
#define ANGLE_ZERO_DEG       0.0F     /* Zero degrees */
#define ANGLE_90_DEG         90.0F    /* 90 degrees (right angle) */
#define ANGLE_180_DEG        180.0F   /* 180 degrees (straight angle) */
#define ANGLE_360_DEG        360.0F   /* 360 degrees (full circle) */

/* Array size constants */
#define ARRAY_SIZE_3         3        /* 3D vector/matrix dimension */

/****************************************************************************
 * Section Name (e.g., Helper Functions, Main Functions, etc.)
 ****************************************************************************/

/**
 * @brief Brief description
 *
 * Detailed description of what the function does, algorithm used, etc.
 *
 * @param[in] param1 - Description with type, units, range
 * @param[out] result - Description of output
 * @return Description of return value
 *
 * @note Any special notes, warnings, or limitations
 */
void function_name(type param1, type *result)
{
    // Implementation using named constants
    for (uint32_t i = 0; i < ARRAY_SIZE_3; i++) {
        result[i] = param1 * DOUBLE_TWO;
    }
}
```

## File-Specific Guidelines

### For algo_sf_quatmath.c/h

**Key Constants to Define:**
```c
/* Quaternion scaling constants */
#define QUAT_SCALE_TWO           2.0      /* Factor for quaternion to rotation matrix */
#define QUAT_HALF_ANGLE          0.5      /* Half angle for sin/cos computations */
#define QUAT_QUARTER             0.25     /* Quarter value for matrix elements */

/* Quaternion thresholds */
#define QUAT_NORMALIZE_MIN       1e-9     /* Minimum magnitude for normalization */
#define QUAT_IDENTITY_SCALAR     1.0      /* Identity quaternion scalar part */
#define QUAT_IDENTITY_VECTOR     0.0      /* Identity quaternion vector parts */

/* Angle constants specific to quaternion math */
#define MAX_POS_PITCH_DEG        179.9999 /* Maximum positive pitch angle */
```

**Functions to Document:**
1. `Quat2RotMtx` - Quaternion to rotation matrix
2. `RotMtx2Quat` - Rotation matrix to quaternion
3. `QuatNormal` - Normalize quaternion
4. `QuatProduct` - Quaternion multiplication
5. `QuatIntegrate` - Time integration (zeroth order)
6. `QuatIntegrate1st` - Time integration (first order)
7. `RotMtx2Angles` - Extract Euler angles
8. `atan2_safe` - Safe atan2 with input checking

### For algo_sf_matrixmath.c

**Key Constants to Define:**
```c
/* Matrix dimensions */
#define MATRIX_DIM_3             3        /* 3x3 matrix dimension */

/* Matrix initialization values */
#define MATRIX_ZERO_DBL          0.0      /* Zero for double precision matrices */
#define MATRIX_ONE_DBL           1.0      /* Identity diagonal element */

/* Matrix printing constants */
#define MATRIX_PRINT_FORMAT      "%f    " /* Printf format for matrix elements */
```

**Functions to Document:** All 12 functions from the header

### For algo_sf_orientation.c

**Key Constants Already Defined:**
- `SMALLQ0`, `CORRUPTQUAT`, `SMALLMODULUS` (already at top)

**Additional Constants Needed:**
```c
/* Rotation angle constants */
#define ROT_ANGLE_THRESHOLD_02   0.02F    /* MacLaurin 3rd order threshold */
#define ROT_ANGLE_THRESHOLD_06   0.06F    /* MacLaurin 5th order threshold */
#define ROT_TRACE_MIN            -1.0F    /* Minimum rotation matrix trace */
#define ROT_TRACE_MAX            3.0F     /* Maximum rotation matrix trace */

/* Quaternion conversion constants */
#define QUAT_SCALE_FACTOR        2.0F     /* Quaternion scaling for matrix */
```

### For algo_sf_6x_sensor_fusion.c and algo_sf_9x_sensor_fusion.c

**Note:** These files likely have many Kalman filter constants. Group them logically:

```c
/****************************************************************************
 * Kalman Filter Process Noise Constants
 ****************************************************************************/

/****************************************************************************
 * Kalman Filter Measurement Noise Constants
 ****************************************************************************/

/****************************************************************************
 * Kalman Filter Update Constants
 ****************************************************************************/

/****************************************************************************
 * Sensor Timing Constants
 ****************************************************************************/
```

### For algo_sf_approxmath.c

**Constants Already in Header:**
- `TAN15DEG`, `TAN30DEG`

**Additional Constants:**
```c
/* Range thresholds for approximations */
#define SINCOS_PERIOD_2PI        6.2831853071795864769F  /* 2*PI */
#define SINCOS_HALF_PERIOD       3.1415926535897932385F  /* PI */

/* Approximation coefficients */
#define ASIN_COEFF_1             /* Define based on implementation */
#define ASIN_COEFF_2             /* Define based on implementation */
```

## Common Patterns to Look For

### Pattern 1: Loop Bounds
```c
// Before:
for (i = 0; i < 3; i++)

// After:
for (i = 0; i < MATRIX_DIM_3; i++)
```

### Pattern 2: Angle Comparisons
```c
// Before:
if (angle >= 180.0F)
    angle -= 360.0F;

// After:
if (angle >= ANGLE_180_DEG)
    angle -= ANGLE_360_DEG;
```

### Pattern 3: Initialization Values
```c
// Before:
q[0] = 1.0;
q[1] = 0.0;
q[2] = 0.0;
q[3] = 0.0;

// After:
q[0] = QUAT_IDENTITY_SCALAR;
q[1] = QUAT_IDENTITY_VECTOR;
q[2] = QUAT_IDENTITY_VECTOR;
q[3] = QUAT_IDENTITY_VECTOR;
```

### Pattern 4: Mathematical Operations
```c
// Before:
result = value * 2.0;
half_value = value * 0.5;

// After:
result = value * DOUBLE_TWO;
half_value = value * DOUBLE_HALF;
```

## Verification Checklist

After updating each file, verify:

- [ ] File header present with all required tags
- [ ] All magic numbers converted to constants
- [ ] Constants defined at top with comments
- [ ] Decoration blocks separate major sections
- [ ] Every function has documentation header
- [ ] @param tags for all parameters
- [ ] @return tag for non-void functions
- [ ] Units specified in descriptions
- [ ] No logic changes introduced
- [ ] Code still compiles without errors
- [ ] Consistent formatting maintained

## Priority Order

Process files in this order for maximum impact:

1. **High Priority** (Core algorithm files):
   - algo_sf_quatmath.c/h
   - algo_sf_matrixmath.c
   - algo_sf_orientation.c

2. **Medium-High Priority** (Sensor fusion):
   - algo_sf_6x_sensor_fusion.c
   - algo_sf_9x_sensor_fusion.c
   - algo_sf_sensordata.c

3. **Medium Priority** (Supporting):
   - algo_sf_approxmath.c/h
   - algo_sf_tasks.h

4. **Low Priority** (Application):
   - main.c
   - test_sensor_platform.c

## Common Mistakes to Avoid

1. **Don't convert array indices unnecessarily**
   ```c
   // GOOD: Using existing constants
   vec[CHX], vec[CHY], vec[CHZ]

   // BAD: Creating new constants for obvious indices
   vec[INDEX_0], vec[INDEX_1], vec[INDEX_2]
   ```

2. **Don't change function signatures**
   - Keep all parameter types the same
   - Keep parameter names consistent between .h and .c

3. **Don't remove existing comments**
   - Preserve all inline comments
   - Add documentation, don't replace

4. **Don't change floating point precision**
   ```c
   // GOOD: Maintain existing precision
   #define VALUE_F    1.0F     // Float
   #define VALUE_D    1.0      // Double

   // BAD: Changing precision
   #define VALUE_F    1.0      // Changed from 1.0F
   ```

5. **Don't create constants for one-time use**
   - Only create constants for values used multiple times
   - Or for values with special meaning that needs documentation

## Testing After Changes

```bash
# Compile to check for errors
cd /Users/vikasyadav/onedrive/GitHub/SensorFusion_9axis
make clean
make all

# Check for remaining magic numbers
grep -rn "90\.0\|180\.0\|360\.0" code/algo/src/*.c | grep -v "#define"

# Verify all functions have headers
for file in code/algo/src/*.c; do
    echo "=== $file ==="
    grep -B2 "^void\|^float\|^double\|^uint32_t" "$file" | head -20
done
```

## Reference: Completed Files

Study these files for examples:
1. `/code/algo/inc/algo_sf_matrix.h` - Simple header example
2. `/code/algo/src/algo_sf_matrix.c` - Simple source example
3. `/code/algo/inc/algo_sf_approx.h` - Complex header example
4. `/code/algo/src/algo_sf_approx.c` - Complex source example with many constants
5. `/code/algo/inc/algo_sf_orientation.h` - Large header with many functions

---

**Last Updated**: 2025-10-13
**Purpose**: Guide for completing documentation updates
