
/*
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// longRes = intIn1 * intIn2
uint_fast32_t MulU16X16to32(uint_fast16_t a, uint_fast16_t b);

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 32bit result
uint_fast16_t MulU16X16toH16(uint_fast16_t a, uint_fast16_t b);

// intRes = intIn1 * intIn2 >> 16 + round
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 32bit result
// 21 cycles
uint_fast16_t MulU16X16toH16Round(uint_fast16_t a, uint_fast16_t b);

// signed16 * signed16
// 22 cycles
int_fast32_t MulS16X16to32(int_fast16_t a, int_fast16_t b);

// signed16 * signed 16 >> 16
int_fast16_t MulS16X16toH16(int_fast16_t a, int_fast16_t b);

// multiplies a signed and unsigned 16 bit ints with a 32 bit result
int_fast32_t MulSU16X16to32(int_fast16_t a, uint_fast16_t b);

// multiplies signed x unsigned int and returns the highest 16 bits of the result
int_fast16_t MulSU16X16toH16(int_fast16_t a, uint_fast16_t b);

// multiplies signed x unsigned int and returns the highest 16 bits of the result
// rounds the result based on the MSB of the lower 16 bits
// 22 cycles
int_fast16_t MulSU16X16toH16Round(int_fast16_t a, uint_fast16_t b);