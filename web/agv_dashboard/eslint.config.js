import js from '@eslint/js'
import globals from 'globals'
import reactHooks from 'eslint-plugin-react-hooks'
import reactRefresh from 'eslint-plugin-react-refresh'
import tseslint from 'typescript-eslint'
import { defineConfig, globalIgnores } from 'eslint/config'

export default defineConfig([
  globalIgnores(['dist']),
  {
    files: ['**/*.{ts,tsx}'],
    extends: [
      js.configs.recommended,
      tseslint.configs.recommended,
      reactHooks.configs.flat.recommended,
      reactRefresh.configs.vite,
    ],
    languageOptions: {
      ecmaVersion: 2020,
      globals: globals.browser,
    },
    rules: {
      // Compound components (Tile) and context hooks (useRobot/useToast) are
      // intentional shared modules — keep the rule for accidental non-component
      // exports but allow the established patterns.
      'react-refresh/only-export-components': [
        'error',
        { allowConstantExport: true, allowExportNames: ['useRobot', 'useToast', 'Tile'] },
      ],
      // Fetch-on-mount / timer / derived-time patterns used throughout the HMI.
      // These react-hooks rules flag legitimate operator-UI code; keep purity of
      // render free of the noise so the real type/unused debt stays visible.
      'react-hooks/set-state-in-effect': 'off',
      'react-hooks/purity': 'off',
      'react-hooks/refs': 'off',
    },
  },
])
