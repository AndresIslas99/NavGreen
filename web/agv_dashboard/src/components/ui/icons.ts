/**
 * Centralized icon imports.
 *
 * We re-export every Lucide icon we actually use from this single barrel so
 * the rest of the app does `import { Circle, Battery } from './ui/icons'` —
 * tree-shaking works because each component file in `lucide-react` is its
 * own ESM module.
 *
 * Adding a new icon? Append a single line here, then import from
 * './ui/icons' in your component. Never `import { X } from 'lucide-react'`
 * directly — that pulls the entire library.
 */
export {
  // State (robot)
  Circle,
  Play,
  MapPin,
  AlertOctagon,
  AlertTriangle,
  CheckCircle2,
  Check,
  Power,
  Activity,

  // Battery
  Battery,
  BatteryLow,
  BatteryWarning,
  BatteryFull,
  BatteryCharging,
  Zap,

  // Localization
  Compass,
  LocateOff,
  Loader,
  AlertCircle,

  // Actions
  Pause,
  Home,
  XOctagon,
  RotateCcw,
  Square,

  // Teleop / map
  Move,
  Crosshair,
  Gamepad,
  Gamepad2,

  // Mode rail
  LayoutGrid,
  Map as MapIcon,
  Route,
  Wrench,
  BarChart3,
  Tag,

  // Top bar / status
  WifiOff,
  Wifi,
  User,
  LogOut,
  LogIn,

  // Navigation / chevrons
  ChevronRight,
  ChevronDown,
  ChevronLeft,
  ChevronUp,
  X,
  Plus,
  Minus,
} from 'lucide-react';

export type { LucideIcon } from 'lucide-react';
