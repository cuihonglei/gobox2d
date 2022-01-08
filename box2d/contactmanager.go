package box2d

// Delegate of b2World.
type ContactManager struct {
	BroadPhase      *BroadPhase
	ContactList     IContact
	ContactCount    int32
	ContactFilter   IContactFilter
	ContactListener IContactListener
}

func NewContactManager() *ContactManager {
	this := new(ContactManager)
	this.BroadPhase = NewBroadPhase()
	this.ContactFilter = NewContactFilter()
	this.ContactListener = NewContactListener()
	return this
}

// Broad-phase callback.
func (this *ContactManager) AddPair(proxyUserDataA interface{}, proxyUserDataB interface{}) {
	proxyA := proxyUserDataA.(*FixtureProxy)
	proxyB := proxyUserDataB.(*FixtureProxy)

	fixtureA := proxyA.Fixture
	fixtureB := proxyB.Fixture

	indexA := proxyA.ChildIndex
	indexB := proxyB.ChildIndex

	bodyA := fixtureA.GetBody()
	bodyB := fixtureB.GetBody()

	// Are the fixtures on the same body?
	if bodyA == bodyB {
		return
	}

	// TODO_ERIN use a hash table to remove a potential bottleneck when both
	// bodies have a lot of contacts.
	// Does a contact already exist?
	edge := bodyB.GetContactList()
	for edge != nil {
		if edge.Other == bodyA {
			fA := edge.Contact.GetFixtureA()
			fB := edge.Contact.GetFixtureB()
			iA := edge.Contact.GetChildIndexA()
			iB := edge.Contact.GetChildIndexB()

			if fA == fixtureA && fB == fixtureB && iA == indexA && iB == indexB {
				// A contact already exists.
				return
			}

			if fA == fixtureB && fB == fixtureA && iA == indexB && iB == indexA {
				// A contact already exists.
				return
			}
		}

		edge = edge.Next
	}

	// Does a joint override collision? Is at least one body dynamic?
	if !bodyB.ShouldCollide(bodyA) {
		return
	}

	// Check user filtering.
	if this.ContactFilter != nil && !this.ContactFilter.ShouldCollide(fixtureA, fixtureB) {
		return
	}

	// Call the factory.
	c := Contact_Create(fixtureA, indexA, fixtureB, indexB)
	if c == nil {
		return
	}

	// Contact creation may swap fixtures.
	fixtureA = c.GetFixtureA()
	fixtureB = c.GetFixtureB()
	indexA = c.GetChildIndexA()
	indexB = c.GetChildIndexB()
	bodyA = fixtureA.GetBody()
	bodyB = fixtureB.GetBody()

	// Insert into the world.
	c.SetPrev(nil)
	c.SetNext(this.ContactList)
	if this.ContactList != nil {
		this.ContactList.SetPrev(c)
	}
	this.ContactList = c

	// Connect to island graph.

	// Connect to body A
	c.GetNodeA().Contact = c
	c.GetNodeA().Other = bodyB

	c.GetNodeA().Prev = nil
	c.GetNodeA().Next = bodyA.contactList
	if bodyA.contactList != nil {
		bodyA.contactList.Prev = c.GetNodeA()
	}
	bodyA.contactList = c.GetNodeA()

	// Connect to body B
	c.GetNodeB().Contact = c
	c.GetNodeB().Other = bodyA

	c.GetNodeB().Prev = nil
	c.GetNodeB().Next = bodyB.contactList
	if bodyB.contactList != nil {
		bodyB.contactList.Prev = c.GetNodeB()
	}
	bodyB.contactList = c.GetNodeB()

	// Wake up the bodies
	bodyA.SetAwake(true)
	bodyB.SetAwake(true)

	this.ContactCount++
}

func (this *ContactManager) FindNewContacts() {
	this.BroadPhase.UpdatePairs(this.AddPair)
}

func (this *ContactManager) Destroy(c IContact) {
	fixtureA := c.GetFixtureA()
	fixtureB := c.GetFixtureB()
	bodyA := fixtureA.GetBody()
	bodyB := fixtureB.GetBody()

	if this.ContactListener != nil && c.IsTouching() {
		this.ContactListener.EndContact(c)
	}

	// Remove from the world.
	if c.GetPrev() != nil {
		c.GetPrev().SetNext(c.GetNext())
	}

	if c.GetNext() != nil {
		c.GetNext().SetPrev(c.GetPrev())
	}

	if c == this.ContactList {
		this.ContactList = c.GetNext()
	}
	// Remove from body 1
	if c.GetNodeA().Prev != nil {
		c.GetNodeA().Prev.Next = c.GetNodeA().Next
	}

	if c.GetNodeA().Next != nil {
		c.GetNodeA().Next.Prev = c.GetNodeA().Prev
	}

	if c.GetNodeA() == bodyA.contactList {
		bodyA.contactList = c.GetNodeA().Next
	}

	// Remove from body 2
	if c.GetNodeB().Prev != nil {
		c.GetNodeB().Prev.Next = c.GetNodeB().Next
	}

	if c.GetNodeB().Next != nil {
		c.GetNodeB().Next.Prev = c.GetNodeB().Prev
	}

	if c.GetNodeB() == bodyB.contactList {
		bodyB.contactList = c.GetNodeB().Next
	}

	// Call the factory.
	//c.Destroy()
	this.ContactCount--
}

// This is the top level collision call for the time step. Here
// all the narrow phase collision is processed for the world
// contact list.
func (this *ContactManager) Collide() {
	// Update awake contacts.
	c := this.ContactList
	for c != nil {
		fixtureA := c.GetFixtureA()
		fixtureB := c.GetFixtureB()
		indexA := c.GetChildIndexA()
		indexB := c.GetChildIndexB()
		bodyA := fixtureA.GetBody()
		bodyB := fixtureB.GetBody()

		// Is this contact flagged for filtering?
		if (c.GetFlags() & Contact_e_filterFlag) != 0 {
			// Should these bodies collide?
			if !bodyB.ShouldCollide(bodyA) {
				cNuke := c
				c = cNuke.GetNext()
				this.Destroy(cNuke)
				continue
			}

			// Check user filtering.
			if this.ContactFilter != nil && !this.ContactFilter.ShouldCollide(fixtureA, fixtureB) {
				cNuke := c
				c = cNuke.GetNext()
				this.Destroy(cNuke)
				continue
			}

			// Clear the filtering flag.
			c.SetFlags(c.GetFlags() & ^Contact_e_filterFlag)
		}

		activeA := bodyA.IsAwake() && bodyA.itype != StaticBody
		activeB := bodyB.IsAwake() && bodyB.itype != StaticBody

		// At least one body must be awake and it must be dynamic or kinematic.
		if !activeA && !activeB {
			c = c.GetNext()
			continue
		}

		proxyIdA := fixtureA.Proxies[indexA].ProxyId
		proxyIdB := fixtureB.Proxies[indexB].ProxyId
		overlap := this.BroadPhase.TestOverlap(proxyIdA, proxyIdB)

		// Here we destroy contacts that cease to overlap in the broad-phase.
		if !overlap {
			cNuke := c
			c = cNuke.GetNext()
			this.Destroy(cNuke)
			continue
		}

		// The contact persists.
		c.Update(this.ContactListener)
		c = c.GetNext()
	}
}
