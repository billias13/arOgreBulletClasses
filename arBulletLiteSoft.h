#include <string>

#include <Ogre.h>
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>

#include "arBulletLite.h"
#include "arBulletConvert.h"

#include "oGlut.h"


namespace arBulletLite
{
		struct	ImplicitSphere : btSoftBody::ImplicitFn
		{
			btVector3	center;
			btScalar	sqradius;
			ImplicitSphere() {}
			ImplicitSphere(const btVector3& c,btScalar r) : center(c),sqradius(r*r) {}
			btScalar	Eval(const btVector3& x)
			{
				return((x-center).length2()-sqradius);
			}
		};


	class SoftObject
	{	
		friend class Scene;
		public:
			Ogre::String mName;

			Ogre::SceneNode*	mNode;	
			Ogre::Entity*		mEntity;
			Ogre::String		mMaterial;
	
			btSoftBodyWorldInfo	mSoftBodyWorldInfo; //Soft body world info
			btSoftBody*			mSoftBody; //Soft body
			int					mTotalNodes; //Total number of nodes

			arBulletLite::Scene* mBtScene;
			Ogre::SceneManager*  mSceneMgr;
			Ogre::SceneNode*	 mParentNode;

			bool mMeshExists;

			SoftObject(const Ogre::String name, arBulletLite::Scene *btScene, Ogre::SceneManager* sceneMgr, Ogre::SceneNode* parentNode)
			{
				//Set object name
				mName = name;
				
				//Create ogre entity and node
				mSceneMgr = sceneMgr;
				mParentNode = parentNode;
				

				//Provide an existing dispatcher and broadphase
				mBtScene = btScene;
				btVector3 gravity = GRAVITY_SCALING*mBtScene->getDynamicsWorld()->getGravity();
				mSoftBodyWorldInfo.m_gravity.setValue(gravity.getX(),gravity.getY(),gravity.getZ());
				
				mBtScene->getDynamicsWorld()->getDispatchInfo().m_enableSPU = true;
				mSoftBodyWorldInfo.m_dispatcher = mBtScene->getCollisionDispatcher();
				mSoftBodyWorldInfo.m_sparsesdf.Reset();
				mSoftBodyWorldInfo.m_broadphase = mBtScene->getBroadphase();
				mSoftBodyWorldInfo.m_sparsesdf.Initialize();

				//Create soft body and add it to world
				mSoftBody = createSoftBody();
				applySoftBodyProperties();
				mBtScene->getDynamicsWorld()->addSoftBody(mSoftBody);

				//Create visual representation of soft body and add to world
				mMeshExists = createOgreMesh();

			}


			~SoftObject()
			{
				mNode->detachAllObjects();
				mSceneMgr->destroyEntity(mEntity);
				mParentNode->removeAndDestroyChild(mNode->getName());
				mBtScene->getDynamicsWorld()->removeSoftBody(mSoftBody);
			}

			void SoftObject::update()
			{
				if(mMeshExists) updateOgreMesh();
			}

			bool SoftObject::cut(int ent)
			{

					ImplicitSphere	isphere(mSoftBody->m_nodes[ent].m_x,5);
					mSoftBody->refine(&isphere,0.0001,true);
					return true;
			}

		private:
		//VIRTUAL FUNCTIONS HERE MANAGE THE SOFT BODY CREATION, MESH CREATION AND MESH UPDATE.
	    //ANY SOFT OBJECT CAN BE DESIGNED USING THIS CLASS TO INHERIT GENERAL METHODS&PROPERTIES
		//AND APPLY CHANGES TO THESE VIRTUAL FUNCTIONS TO CHANGE THE SHAPE OF THE OBJECT

			virtual btSoftBody* SoftObject::createSoftBody()
			{
				btScalar s = 40; btScalar h = 40; int resolution = 10;
				mTotalNodes = resolution*resolution;
				const btVector3	p[]={	btVector3(+s,-s,h), btVector3(-s,-s,h), btVector3(+s,+s,h), btVector3(-s,+s,h)};
				btSoftBody* psb = btSoftBodyHelpers::CreatePatch(mSoftBodyWorldInfo,p[0],p[1],p[2],p[3],resolution,resolution, 1+2+4+8,true);
				//btSoftBody* psb = btSoftBodyHelpers::CreatePatch(mSoftBodyWorldInfo,p[0],p[1],p[2],p[3],resolution,resolution, 0,true);
				return  psb;
				//return btSoftBodyHelpers::CreatePatch(mSoftBodyWorldInfo,p[0],p[1],p[2],p[3],resolution,resolution, 0,true);
			}


			virtual void SoftObject::applySoftBodyProperties()
			{
					//mSoftBody->m_cfg.piterations = 2;
					//mSoftBody->m_cfg.citerations = 2;
					//mSoftBody->m_cfg.diterations = 2;
					//mSoftBody->m_cfg.kDF			=	0.5; // Dynamic friction coefficient 
					//mSoftBody->m_cfg.kCHR = 0.1;			 // Rigid contacts hardness 

				btSoftBody::Material*	pm=mSoftBody->appendMaterial();
					pm->m_kLST = 0.4;
				/*	pm->m_kAST = 0.2;
					pm->m_kVST = 0.5;*/

			/*		mSoftBody->m_cfg.collisions	|=	btSoftBody::fCollision::RVSmask + btSoftBody::fCollision::SDF_RS + 
												btSoftBody::fCollision::CL_SS +
												btSoftBody::fCollision::CL_SELF;*/
					
					mSoftBody->m_cfg.collisions  |= btSoftBody::fCollision::VF_SS;

				mSoftBody->setTotalMass( 150, false ); 
				//mSoftBody->generateClusters(20);
				//mSoftBody->generateBendingConstraints(2,pm);
			}


			virtual bool SoftObject::createOgreMesh()
			{
				int resolution = (int)sqrt((float)mTotalNodes);
				/* create the mesh and a single sub mesh */
				Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual("CustomMesh", "General");
				Ogre::SubMesh *subMesh = mesh->createSubMesh();

				/* create the vertex data structure */
				mesh->sharedVertexData = new Ogre::VertexData;
				mesh->sharedVertexData->vertexCount = mTotalNodes;

				/* declare how the vertices will be represented */
				Ogre::VertexDeclaration *decl = mesh->sharedVertexData->vertexDeclaration;
				size_t offset = 0;

				/* the first three floats of each vertex represent the position */
				decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
				offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

				/* the second three floats of each vertex represent the colour */
				decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
				offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

				// two dimensional texture coordinates
				 decl->addElement(0, offset, VET_FLOAT2, VES_TEXTURE_COORDINATES, 0);
				 offset += VertexElement::getTypeSize(VET_FLOAT2);

				/* create the vertex buffer */
				Ogre::HardwareVertexBufferSharedPtr vertexBuffer = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(offset, mTotalNodes, Ogre::HardwareBuffer::HBU_STATIC);

				/* lock the buffer so we can get exclusive access to its data */
				float *vertices = static_cast<float *>(vertexBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL));

				for(int i = 0;i<mTotalNodes;++i)
				{
					float x = mSoftBody->m_nodes[i].m_x.getX();
					float y = mSoftBody->m_nodes[i].m_x.getY();
					float z = mSoftBody->m_nodes[i].m_x.getZ();
										
					/* position */
					vertices[i*8]	= x;
					vertices[i*8+1] = y;
					vertices[i*8+2] = z;
					/* normals */
					vertices[i*8+3]= mSoftBody->m_nodes[i].m_n.getX();
					vertices[i*8+4]= mSoftBody->m_nodes[i].m_n.getY();
					vertices[i*8+5]= mSoftBody->m_nodes[i].m_n.getZ();

					int row = (int)i/resolution; float tx =  (float)row/ (float)(resolution-1);
					int column = (int)i%resolution; float ty =  (float)column/ (float)(resolution-1);
					vertices[i*8+6] = tx;
					vertices[i*8+7] = ty;	
				}

				/* unlock the buffer */
				vertexBuffer->unlock();

	
				//Ogre::HardwareIndexBufferSharedPtr indexBuffer = Ogre::HardwareBufferManager::getSingleton().
				//createIndexBuffer(Ogre::HardwareIndexBuffer::IT_16BIT, mesh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC);
				
				int totalTriangles = 2*(resolution-1)*(resolution-1);
				int addedTriangles = 0;
				int sizeOfTriangle = 3;
				int totalNodes = mTotalNodes;

				/* create the index buffer */
				Ogre::HardwareIndexBufferSharedPtr indexBuffer = Ogre::HardwareBufferManager::getSingleton().
																 createIndexBuffer(Ogre::HardwareIndexBuffer::IT_16BIT, 
																				   totalTriangles*3 /*mesh->sharedVertexData->vertexCount*/, 
																				   Ogre::HardwareBuffer::HBU_STATIC);

				/* lock the buffer so we can get exclusive access to its data */
				uint16_t *indices = static_cast<uint16_t *>(indexBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL));


				for (int node = 0; node<(mTotalNodes-resolution); node++)
				{
					if((node+1)%resolution!=0)
					{
						/* define triangles */
						indices[(addedTriangles*sizeOfTriangle)]	 = node;
						indices[(addedTriangles*sizeOfTriangle) + 1] = node+resolution;
						indices[(addedTriangles*sizeOfTriangle) + 2] = node+1;
					//	cout<<addedTriangles<<":"<<node<<"  "<<node+resolution<<"   "<<node+1<<endl;
						addedTriangles++;

						indices[(addedTriangles*sizeOfTriangle)]	 = node+1;
						indices[(addedTriangles*sizeOfTriangle) + 1] = node+resolution;
						indices[(addedTriangles*sizeOfTriangle) + 2] = node+resolution+1; 
					//	cout<<addedTriangles<<":"<<node+resolution<<"  "<<node+resolution+1<<"   "<<node+1<<endl;
						addedTriangles++;
					}
				}



					/* unlock the buffer */
					indexBuffer->unlock();

					/* attach the buffers to the mesh */
					mesh->sharedVertexData->vertexBufferBinding->setBinding(0, vertexBuffer);
					subMesh->useSharedVertices = true;
					subMesh->indexData->indexBuffer = indexBuffer;
					subMesh->indexData->indexCount = totalTriangles*3;
					subMesh->indexData->indexStart = 0;

					/* set the bounds of the mesh */
					mesh->_setBounds(Ogre::AxisAlignedBox(-100, -100, -100,100,100,100));

					/* notify the mesh that we're all ready */
					mesh->load();

					/* you can now create an entity/scene node based on your mesh, e.g. */
					mEntity = mSceneMgr->createEntity(mName, "CustomMesh", "General");
					mEntity->setMaterialName("Examples/OgreLogo");
					mNode = mParentNode->createChildSceneNode();
					mNode->attachObject(mEntity);

					return true;
				}

				virtual void SoftObject::updateOgreMesh()
				{
				
					MeshPtr mesh= mEntity->getMesh();
					Ogre::SubMesh* submesh = mesh->getSubMesh(0);
					Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;

					if (submesh->useSharedVertices)
					{
						const Ogre::VertexElement* posElem =vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
						Ogre::HardwareVertexBufferSharedPtr vbuf= vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());
						const unsigned int vSize = (unsigned int)vbuf->getVertexSize();
						unsigned char* vertex =static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

						float* pReal;

						//get the vertex data from bullet.
						btSoftBody::tNodeArray&   nodes(mSoftBody->m_nodes);

						//now we simply update the coords of the vertices. Of course this only works
						//properly when the Ogre data and the Bullet data have the same number of vertices
						//and when they are in the same order. It seems that bullet doesn't shuffle the
						//vertices, but this is only an observation!
						for (int j=0;j<nodes.size();++j)
						{

							posElem->baseVertexPointerToElement(vertex, &pReal);
							vertex += vSize;
							*pReal++= nodes[j].m_x.x();
							*pReal++= nodes[j].m_x.y();
							*pReal++= nodes[j].m_x.z();

							*pReal++= nodes[j].m_n.getX();
							*pReal++= nodes[j].m_n.getY();
							*pReal++= nodes[j].m_n.getZ();
							*pReal++;
							*pReal++;
						}


						vbuf->unlock();
					}
			}
	};


}