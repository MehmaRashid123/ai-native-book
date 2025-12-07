import React, { useState } from 'react';

interface User {
  id: number;
  email: string;
  software_skill: string;
  hardware_access: boolean;
}

interface AuthResponse {
  access_token: string;
  token_type: string;
  user: User;
}

interface AuthModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSuccess: (token: string, user: User) => void;
  isLoginMode: boolean; // Added this prop to match usage in AuthContext
}

const API_BASE_URL = 'http://localhost:8000';

const AuthModal: React.FC<AuthModalProps> = ({ isOpen, onClose, onSuccess, isLoginMode }) => {
  // Use prop to set initial state, but allow internal toggling
  const [isLogin, setIsLogin] = useState(isLoginMode);
  
  // Sync state if prop changes (optional, depending on UX desired)
  React.useEffect(() => {
    setIsLogin(isLoginMode);
  }, [isLoginMode]);

  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [softwareSkill, setSoftwareSkill] = useState('Beginner');
  const [hardwareAccess, setHardwareAccess] = useState(false);
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setIsLoading(true);

    const endpoint = isLogin ? `${API_BASE_URL}/api/auth/login` : `${API_BASE_URL}/api/auth/signup`;
    
    // Construct body based on mode
    const body: any = { email, password };
    if (!isLogin) {
      body.software_skill = softwareSkill;
      body.hardware_access = hardwareAccess;
    }

    try {
      const response = await fetch(endpoint, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(body),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail || 'Authentication failed');
      }

      const authData = data as AuthResponse;
      onSuccess(authData.access_token, authData.user);
      onClose(); 
    } catch (err: any) {
      setError(err.message || 'An unexpected error occurred.');
    } finally {
      setIsLoading(false);
    }
  };

  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 bg-gray-900 bg-opacity-75 flex items-center justify-center z-[99999999]">
      <div className="bg-gray-800 p-8 rounded-lg shadow-xl w-full max-w-md border border-gray-700 relative">
        <button
            onClick={onClose}
            className="absolute top-4 right-4 text-gray-400 hover:text-white"
            aria-label="Close"
        >
            <svg xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24" strokeWidth={1.5} stroke="currentColor" className="w-6 h-6">
                <path strokeLinecap="round" strokeLinejoin="round" d="M6 18 18 6M6 6l12 12" />
            </svg>
        </button>

        <h2 className="text-2xl font-bold text-white mb-6 text-center">
          {isLogin ? 'Login' : 'Sign Up'}
        </h2>

        {error && (
          <div className="bg-red-500 text-white text-sm p-3 rounded-md mb-4 text-center">
            {error}
          </div>
        )}

        <form onSubmit={handleSubmit} className="space-y-4">
          <div>
            <label className="block text-gray-300 text-sm font-bold mb-2" htmlFor="email">
              Email
            </label>
            <input
              type="email"
              id="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              className="shadow appearance-none border border-gray-600 rounded w-full py-2 px-3 text-gray-100 leading-tight focus:outline-none focus:shadow-outline bg-gray-700"
              required
            />
          </div>
          <div>
            <label className="block text-gray-300 text-sm font-bold mb-2" htmlFor="password">
              Password
            </label>
            <input
              type="password"
              id="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              className="shadow appearance-none border border-gray-600 rounded w-full py-2 px-3 text-gray-100 leading-tight focus:outline-none focus:shadow-outline bg-gray-700"
              required
            />
          </div>

          {!isLogin && (
            <>
              <div>
                <label className="block text-gray-300 text-sm font-bold mb-2" htmlFor="softwareSkill">
                  Software Skill
                </label>
                <select
                  id="softwareSkill"
                  value={softwareSkill}
                  onChange={(e) => setSoftwareSkill(e.target.value)}
                  className="shadow border border-gray-600 rounded w-full py-2 px-3 text-gray-100 leading-tight focus:outline-none focus:shadow-outline bg-gray-700"
                >
                  <option value="Beginner">Beginner</option>
                  <option value="Intermediate">Intermediate</option>
                  <option value="Advanced">Advanced</option>
                </select>
              </div>
              <div className="flex items-center">
                <input
                  type="checkbox"
                  id="hardwareAccess"
                  checked={hardwareAccess}
                  onChange={(e) => setHardwareAccess(e.target.checked)}
                  className="mr-2 h-4 w-4 text-blue-600 focus:ring-blue-500 border-gray-600 rounded bg-gray-700"
                />
                <label className="text-gray-300 text-sm" htmlFor="hardwareAccess">
                  I have access to physical robotics hardware.
                </label>
              </div>
            </>
          )}

          <button
            type="submit"
            className="w-full bg-blue-600 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded focus:outline-none focus:shadow-outline transition duration-150 ease-in-out mt-4"
            disabled={isLoading}
          >
            {isLoading ? 'Processing...' : (isLogin ? 'Login' : 'Sign Up')}
          </button>
        </form>

        <p className="text-center text-gray-400 text-sm mt-6">
          {isLogin ? "Don't have an account?" : "Already have an account?"}{' '}
          <button
            onClick={() => setIsLogin(!isLogin)}
            className="text-blue-400 hover:text-blue-300 font-bold focus:outline-none"
          >
            {isLogin ? 'Sign Up' : 'Login'}
          </button>
        </p>
      </div>
    </div>
  );
};

export default AuthModal;