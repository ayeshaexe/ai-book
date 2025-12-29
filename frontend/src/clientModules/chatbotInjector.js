import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import FloatingChatbot from '../components/FloatingChatbot/FloatingChatbot';

export default function Root({ children }) {
  return (
    <>
      {children}
      {ExecutionEnvironment.canUseDOM && <FloatingChatbot />}
    </>
  );
}