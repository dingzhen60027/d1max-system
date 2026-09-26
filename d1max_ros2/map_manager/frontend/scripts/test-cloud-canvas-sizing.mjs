import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

test('cloud canvas CSS size follows its host independently of the high-DPI drawing buffer', () => {
  const css = readFileSync(new URL('../src/styles.css', import.meta.url), 'utf8')
  const rule = css.match(/\.cloud-host canvas\s*\{([^}]+)\}/)?.[1]
  assert.ok(rule, 'Point-cloud canvas needs its own sizing rule')
  assert.match(rule, /(?:^|;)\s*width:\s*100%\s*(?:;|$)/)
  assert.match(rule, /(?:^|;)\s*height:\s*100%\s*(?:;|$)/)
  assert.match(rule, /(?:^|;)\s*display:\s*block\s*(?:;|$)/)
})
