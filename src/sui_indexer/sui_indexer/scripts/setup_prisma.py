#!/usr/bin/env python3
import os
import subprocess
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node

class PrismaSetupNode(Node):
    def __init__(self):
        super().__init__('prisma_setup')
        
        # Get package path
        self.pkg_path = str(Path(__file__).parent.parent.parent)
        self.prisma_path = os.path.join(self.pkg_path, 'prisma')
        
        # Set database URL
        os.environ['DATABASE_URL'] = 'file:sui_indexer.db'
        
        # Run setup
        self.setup_prisma()
        
    def setup_prisma(self):
        """Run Prisma generate and migrate."""
        try:
            # Generate Prisma client
            self.get_logger().info('Generating Prisma client...')
            subprocess.run(['prisma', 'generate'], 
                         cwd=self.prisma_path, 
                         check=True)
            
            # Push the schema to the database
            self.get_logger().info('Pushing schema to database...')
            subprocess.run(['prisma', 'db', 'push', '--accept-data-loss'], 
                         cwd=self.prisma_path, 
                         check=True)
            
            self.get_logger().info('Prisma setup completed successfully')
            
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Prisma setup failed: {str(e)}')
            sys.exit(1)
        except Exception as e:
            self.get_logger().error(f'Unexpected error during Prisma setup: {str(e)}')
            sys.exit(1)

def main(args=None):
    rclpy.init(args=args)
    node = PrismaSetupNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 