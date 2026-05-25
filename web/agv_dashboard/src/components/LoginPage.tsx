import { useState } from 'react'
import * as api from '../api/client'
import { Card, Button } from './ui'
import { LogIn } from './ui/icons'

interface Props {
  onLogin: (username: string, role: string) => void
}

// Tiny inline leaf logo — same shape as the topbar brand mark.
function BrandLogo() {
  return (
    <svg width="40" height="40" viewBox="0 0 28 28" aria-hidden="true">
      <path
        d="M14 3 C 8 5 4 10 5 17 C 5 22 9 25 14 25 C 19 25 23 22 23 17 C 24 10 20 5 14 3 Z"
        fill="var(--accent-soft)" stroke="var(--accent)" strokeWidth="1.5"
      />
      <path d="M14 6 L14 24" stroke="var(--accent)" strokeWidth="1.2" opacity="0.65" />
    </svg>
  )
}

export function LoginPage({ onLogin }: Props) {
  const [username, setUsername] = useState('')
  const [password, setPassword] = useState('')
  const [error, setError] = useState('')
  const [loading, setLoading] = useState(false)

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault()
    setError('')
    setLoading(true)
    try {
      const result = await api.login(username, password)
      if (result.token) {
        api.setToken(result.token)
        onLogin(result.username, result.role)
      } else {
        setError('Credenciales inválidas')
      }
    } catch {
      setError('No se pudo iniciar sesión')
    }
    setLoading(false)
  }

  return (
    <div className="login-page">
      <Card padding="spacious" shadow="md" className="login-card">
        <header className="login-card__header">
          <BrandLogo />
          <div className="login-card__brand">
            <h1 className="login-card__title">AGV-01</h1>
            <p className="login-card__sub">Greenhouse 1 · Operator dashboard</p>
          </div>
        </header>

        <form className="login-form" onSubmit={handleSubmit}>
          <label className="login-form__label" htmlFor="login-user">Usuario</label>
          <input
            id="login-user"
            className="login-form__input"
            type="text"
            value={username}
            onChange={e => setUsername(e.target.value)}
            autoFocus
            autoComplete="username"
          />

          <label className="login-form__label" htmlFor="login-pass">Contraseña</label>
          <input
            id="login-pass"
            className="login-form__input"
            type="password"
            value={password}
            onChange={e => setPassword(e.target.value)}
            autoComplete="current-password"
          />

          {error && <p className="login-form__error">{error}</p>}

          <Button
            variant="primary"
            size="lg"
            block
            leadingIcon={LogIn}
            type="submit"
            disabled={!username || !password}
            loading={loading}
          >
            Iniciar sesión
          </Button>
        </form>
      </Card>
    </div>
  )
}
