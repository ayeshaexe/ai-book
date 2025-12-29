import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import FloatingChatbot from '../components/FloatingChatbot/FloatingChatbot';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props}>
        {props.children}
      </OriginalLayout>
      {ExecutionEnvironment.canUseDOM && <FloatingChatbot />}
    </>
  );
}