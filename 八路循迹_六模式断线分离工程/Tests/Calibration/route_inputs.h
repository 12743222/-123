#ifndef TEST_ROUTE_INPUTS_H
#define TEST_ROUTE_INPUTS_H
/* Preserve the original route scenarios as four spatial regions, while the
   real driver receives eight independent active-low electrical inputs. */
static uint16_t route_input_pins(unsigned regions)
{
    uint16_t value=0xffffu;
    if(regions&1u) value^=GPIO_Pin_14|GPIO_Pin_12; /* X1/X2 */
    if(regions&2u) value^=GPIO_Pin_13|GPIO_Pin_15; /* X3/X4 */
    if(regions&4u) value^=GPIO_Pin_5|GPIO_Pin_1;   /* X5/X6 */
    if(regions&8u) value^=GPIO_Pin_10|GPIO_Pin_11; /* X7/X8 */
    return value;
}
#endif
