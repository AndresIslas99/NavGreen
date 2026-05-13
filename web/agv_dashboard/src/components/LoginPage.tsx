import { useState } from 'react'
import * as api from '../api/client'

interface Props {
  onLogin: (username: string, role: string) => void
}

// Sprint E (HIGH-04-04 frontend / CRITICAL-11-C-01). After a successful
// login, if the backend sets must_change_password, we render the
// change-password panel in-place and refuse to call onLogin until the
// user has set a new password. The token from the initial login is
// held in `pendingSession` and only commits to api.setToken() once
// the change succeeds.
export function LoginPage({ onLogin }: Props) {
  const [username, setUsername] = useState('')
  const [password, setPassword] = useState('')
  const [error, setError] = useState('')
  const [loading, setLoading] = useState(false)

  // Forced-change flow state
  const [pendingSession, setPendingSession] = useState<{
    token: string
    username: string
    role: string
  } | null>(null)
  const [newPassword, setNewPassword] = useState('')
  const [confirmPassword, setConfirmPassword] = useState('')

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault()
    setError('')
    setLoading(true)
    try {
      const result = await api.login(username, password)
      if (!result?.token) {
        setError('Invalid credentials')
        setLoading(false)
        return
      }
      if (result.must_change_password) {
        // Hold the new token in component state — do NOT commit to
        // api.setToken yet. The user must set a new password first,
        // otherwise reloading the page would skip the change-password
        // prompt (token already stored = "logged in").
        setPendingSession({
          token: result.token,
          username: result.username,
          role: result.role,
        })
        setLoading(false)
        return
      }
      // Normal path: persist the token and notify the parent.
      api.setToken(result.token)
      onLogin(result.username, result.role)
    } catch {
      setError('Login failed')
      setLoading(false)
    }
  }

  const handleChangeSubmit = async (e: React.FormEvent) => {
    e.preventDefault()
    setError('')
    if (newPassword.length < 8) {
      setError('New password must be at least 8 characters')
      return
    }
    if (newPassword !== confirmPassword) {
      setError('New password and confirmation do not match')
      return
    }
    if (!pendingSession) return
    setLoading(true)
    try {
      const result = await api.changePassword(pendingSession.username, password, newPassword)
      if (!result?.success) {
        setError('Password change failed — backend rejected the request')
        setLoading(false)
        return
      }
      // Now commit the (still-valid) token and notify parent. The
      // backend cleared must_change_password server-side; future logins
      // from this account will skip this branch.
      api.setToken(pendingSession.token)
      onLogin(pendingSession.username, pendingSession.role)
    } catch {
      setError('Password change failed')
      setLoading(false)
    }
  }

  // ── Render: forced-change form once login produced a pending session ──
  if (pendingSession) {
    return (
      <div className="login-page">
        <form className="login-form" onSubmit={handleChangeSubmit}>
          <h2 className="login-title">Set a new password</h2>
          <p style={{ opacity: 0.8, fontSize: 13, marginBottom: 12, lineHeight: 1.4 }}>
            This is the first login for <strong>{pendingSession.username}</strong>.
            The auto-generated initial password must be changed before you can
            use the dashboard. Choose a password that is at least 8 characters
            long.
          </p>
          <input
            type="password"
            placeholder="New password"
            value={newPassword}
            onChange={e => setNewPassword(e.target.value)}
            autoFocus
            minLength={8}
          />
          <input
            type="password"
            placeholder="Confirm new password"
            value={confirmPassword}
            onChange={e => setConfirmPassword(e.target.value)}
            minLength={8}
          />
          {error && <div className="login-error">{error}</div>}
          <button type="submit" disabled={loading || !newPassword || !confirmPassword}>
            {loading ? 'Updating...' : 'Set password and continue'}
          </button>
        </form>
      </div>
    )
  }

  // ── Render: regular login ──
  return (
    <div className="login-page">
      <form className="login-form" onSubmit={handleSubmit}>
        <h2 className="login-title">AGV Control</h2>
        <input
          type="text"
          placeholder="Username"
          value={username}
          onChange={e => setUsername(e.target.value)}
          autoFocus
        />
        <input
          type="password"
          placeholder="Password"
          value={password}
          onChange={e => setPassword(e.target.value)}
        />
        {error && <div className="login-error">{error}</div>}
        <button type="submit" disabled={loading || !username || !password}>
          {loading ? 'Logging in...' : 'Login'}
        </button>
      </form>
    </div>
  )
}
