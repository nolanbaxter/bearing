// The whole test framework: run a function, count it, say so. A failing
// assertion throws, which stops the file with a stack trace and a non-zero
// exit, and that is all CI needs.

let passed = 0;

export function test(name, fn) {
  fn();
  passed++;
  console.log(`  ok  ${name}`);
}

/** For a test that has to wait, such as for the garbage collector. Await it. */
export async function testAsync(name, fn) {
  await fn();
  passed++;
  console.log(`  ok  ${name}`);
}

export function suite(name) {
  console.log(`\n${name}`);
  process.on('exit', (code) => {
    if (code === 0) console.log(`${passed} passed`);
  });
}

/** The message `fn` throws, or a failure if it doesn't throw. */
export function thrown(fn) {
  try {
    fn();
  } catch (e) {
    return e.message;
  }
  throw new Error('expected a throw, and nothing was thrown');
}
